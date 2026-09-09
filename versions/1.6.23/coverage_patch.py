"""Incremental, region-scoped coverage patch planning.

The module deliberately reuses the scientific planner's scene BVH, surface-cell
keys, ray casting, observation accounting and overlap representation. It adds a
local candidate generator and a stopping greedy selector; it does not run or
replace the global path planner.
"""

from __future__ import annotations

import json
import math
import re
from collections import Counter
from dataclasses import dataclass, field

import bpy
from mathutils import Vector

from . import scientific_planner


PREVIEW_COLLECTION = "PatchCameras_Preview"
FINAL_COLLECTION = "PatchCameras_Final"
HISTORY_KEY = "gs_patch_history_json"
PREVIEW_REPORT_KEY = "gs_patch_preview_report_json"
_PREVIEW_PLANS = {}


@dataclass
class PatchRegion:
    name: str
    cell_keys: set
    center: Vector
    low: Vector
    high: Vector
    dominant_category: str


@dataclass
class PatchPlan:
    cache: object
    mode: str
    target_names: list
    target_keys: set
    regions: list
    existing_candidates: list
    existing_coverage: dict
    selected: list
    before: dict
    after: dict
    candidate_count: int
    safe_origin_count: int
    rejected_unsafe_count: int
    report: dict = field(default_factory=dict)


def _clamp(value, low, high):
    return max(low, min(high, value))


def _collection(scene, name):
    collection = bpy.data.collections.get(name)
    if collection is None:
        collection = bpy.data.collections.new(name)
        scene.collection.children.link(collection)
    return collection


def _remove_camera_object(obj):
    for child in list(obj.children):
        if child.get("gs_camera_mesh_visual"):
            bpy.data.objects.remove(child, do_unlink=True)
    data = obj.data if obj.type == "CAMERA" else None
    bpy.data.objects.remove(obj, do_unlink=True)
    if data is not None and not data.users:
        bpy.data.cameras.remove(data)


def clear_preview(scene):
    collection = bpy.data.collections.get(PREVIEW_COLLECTION)
    if collection is not None:
        for obj in list(collection.objects):
            _remove_camera_object(obj)
    _PREVIEW_PLANS.pop(scene.as_pointer(), None)
    if PREVIEW_REPORT_KEY in scene:
        del scene[PREVIEW_REPORT_KEY]


def preview_cameras(scene):
    collection = bpy.data.collections.get(PREVIEW_COLLECTION)
    if collection is None:
        return []
    return sorted(
        (obj for obj in collection.objects if obj.type == "CAMERA" and obj.get("gs_patch_preview")),
        key=lambda obj: obj.name,
    )


def final_cameras(scene):
    collection = bpy.data.collections.get(FINAL_COLLECTION)
    if collection is None:
        return []
    return sorted(
        (obj for obj in collection.objects if obj.type == "CAMERA" and obj.get("gs_patch_camera")),
        key=lambda obj: obj.name,
    )


def _object_contains_point(obj, world_point):
    local = obj.matrix_world.inverted_safe() @ world_point
    if obj.type == "EMPTY":
        half = max(1e-6, float(getattr(obj, "empty_display_size", 1.0)))
        return all(abs(value) <= half + 1e-6 for value in local)
    corners = list(getattr(obj, "bound_box", ()))
    if not corners:
        return False
    low = Vector(tuple(min(corner[axis] for corner in corners) for axis in range(3)))
    high = Vector(tuple(max(corner[axis] for corner in corners) for axis in range(3)))
    return all(low[axis] - 1e-6 <= local[axis] <= high[axis] + 1e-6 for axis in range(3))


def _target_spec(scene, settings, selected_objects):
    mode = getattr(settings, "patch_mode", "SELECTED_OBJECTS")
    if mode == "SELECTED_OBJECTS":
        objects = sorted(
            (
                obj for obj in selected_objects
                if obj.type == "MESH" and not obj.get("gs_camera_mesh_visual")
            ),
            key=lambda obj: obj.name,
        )
        if not objects:
            raise ValueError("请选择至少一个需要补齐的 Mesh 对象")
        names = {obj.name_full for obj in objects}
        return mode, [obj.name for obj in objects], names, None
    if mode == "BOUNDS":
        bounds = getattr(settings, "patch_bounds_object", None)
        if bounds is None:
            raise ValueError("请指定用于补齐的包围盒对象或 Empty")
        return mode, [bounds.name], None, lambda _obj, point: _object_contains_point(bounds, point)
    if mode == "AUTO_UNDEROBSERVED":
        return mode, ["auto_underobserved"], None, None
    raise ValueError(f"未知补齐模式: {mode}")


def _camera_candidate(camera, index, local_step):
    forward = (camera.matrix_world.to_quaternion() @ Vector((0.0, 0.0, -1.0))).normalized()
    yaw = math.atan2(forward.y, forward.x) % math.tau
    pitch = math.asin(_clamp(forward.z, -1.0, 1.0))
    return scientific_planner.Candidate(
        candidate_id=f"PATCH_EXISTING_{index:05d}_{camera.name}",
        origin_id=f"PATCH_EXISTING_{index:05d}",
        component_index=-1,
        station_index=index,
        distance_along=0.0,
        local_step=local_step,
        position=camera.matrix_world.translation.copy(),
        tangent=forward.copy(),
        layer_index=0,
        layer_name="existing",
        yaw=yaw,
        pitch=pitch,
        kind="existing",
    )


def _copy_coverage(coverage):
    result = {}
    for key, value in coverage.items():
        item = dict(value)
        item["view_direction_bins"] = set(value.get("view_direction_bins", ()))
        item["candidate_camera_ids"] = list(value.get("candidate_camera_ids", ()))
        if hasattr(value.get("position"), "copy"):
            item["position"] = value["position"].copy()
        result[key] = item
    return result


def _measure(target_keys, coverage, minimum, recommended):
    total = len(target_keys)
    counts = [coverage.get(key, {}).get("observation_count", 0) for key in target_keys]
    observed = sum(count > 0 for count in counts)
    minimum_met = sum(count >= minimum for count in counts)
    recommended_met = sum(count >= recommended for count in counts)
    diverse = sum(
        len(coverage.get(key, {}).get("view_direction_bins", ())) >= 2
        for key in target_keys
    )
    denominator = max(1, total)
    return {
        "target_surface_cell_count": total,
        "observed_cell_count": observed,
        "under_observed_cell_count": total - minimum_met,
        "under_recommended_cell_count": total - recommended_met,
        "region_coverage_ratio": observed / denominator,
        "minimum_observation_ratio": minimum_met / denominator,
        "recommended_observation_ratio": recommended_met / denominator,
        "diverse_observation_ratio": diverse / denominator,
    }


def _satisfied(metrics, target_ratio):
    return (
        metrics["region_coverage_ratio"] >= target_ratio
        and metrics["minimum_observation_ratio"] >= target_ratio
    )


def _region_from_keys(name, keys, surface_cells):
    cells = [surface_cells[key] for key in keys if key in surface_cells]
    if not cells:
        return None
    positions = [cell["position"] for cell in cells]
    low = Vector(tuple(min(point[axis] for point in positions) for axis in range(3)))
    high = Vector(tuple(max(point[axis] for point in positions) for axis in range(3)))
    center = sum(positions, Vector()) / len(positions)
    categories = Counter(cell["category"] for cell in cells)
    category = categories.most_common(1)[0][0]
    floor_count = categories.get("floor_like", 0)
    ceiling_count = categories.get("ceiling_like", 0)
    extent = high - low
    horizontal_span = max(extent.x, extent.y, 1e-9)
    horizontal_slab = extent.z <= 0.25 * horizontal_span
    vertical_slab = min(extent.x, extent.y) <= 0.25 * max(extent.z, horizontal_span)
    # Only geometrically horizontal thin slabs use candidates on both sides. Surface-cell
    # counts alone over-represent a thin wall's narrow top/bottom faces.
    if horizontal_slab and floor_count and ceiling_count and min(floor_count, ceiling_count) >= 0.75 * max(floor_count, ceiling_count):
        category = "horizontal_both"
    elif vertical_slab and categories.get("vertical_like", 0):
        category = "vertical_like"
    elif not horizontal_slab and categories.get("vertical_like", 0):
        category = "vertical_like"
    return PatchRegion(name, set(keys), center, low, high, category)


def _build_regions(mode, target_names, target_keys, surface_cells):
    if mode == "SELECTED_OBJECTS":
        regions = []
        for name in target_names:
            keys = {key for key in target_keys if surface_cells[key]["object_name"] == name}
            region = _region_from_keys(name, keys, surface_cells)
            if region is not None:
                regions.append(region)
        return regions
    if mode == "AUTO_UNDEROBSERVED":
        by_object = {}
        for key in target_keys:
            by_object.setdefault(surface_cells[key]["object_name"], set()).add(key)
        ranked = sorted(by_object.items(), key=lambda item: (-len(item[1]), item[0]))[:8]
        return [
            region for name, keys in ranked
            if (region := _region_from_keys(name, keys, surface_cells)) is not None
        ]
    region = _region_from_keys(target_names[0], target_keys, surface_cells)
    return [region] if region is not None else []


def _existing_analysis(cache, cameras, settings, progress=None):
    local_step = max(
        float(getattr(settings, "patch_candidate_radius", 1.5)),
        float(getattr(settings, "patch_camera_safety_distance", 0.25)) * 2.0,
    ) * cache.units_per_meter
    candidates = []
    coverage = {}
    maximum_incidence = float(getattr(settings, "scientific_maximum_incidence_angle", 75.0))
    total = max(1, len(cameras))
    analysis_grid = min(cache.ray_grid, 8)
    for index, camera in enumerate(cameras):
        if progress and (index % 8 == 0 or index + 1 == total):
            progress("分析已有相机覆盖", index + 1, total)
        candidate = _camera_candidate(camera, index, local_step)
        scientific_planner._cast_candidate(cache, candidate, ray_grid=analysis_grid)
        scientific_planner._apply_candidate(coverage, candidate, maximum_incidence)
        candidates.append(candidate)
    return candidates, coverage


def _look_angles(origin, target):
    direction = target - origin
    if direction.length <= 1e-8:
        return 0.0, 0.0
    direction.normalize()
    return math.atan2(direction.y, direction.x) % math.tau, math.asin(_clamp(direction.z, -1.0, 1.0))


def _ring_origins(cache, region, radius):
    category = region.dominant_category
    if category == "floor_like":
        z_offsets = (0.55 * radius, 0.9 * radius, 1.25 * radius)
    elif category == "ceiling_like":
        z_offsets = (-1.25 * radius, -0.9 * radius, -0.55 * radius)
    elif category == "horizontal_both":
        z_offsets = (-1.05 * radius, -0.65 * radius, 0.65 * radius, 1.05 * radius)
    else:
        z_offsets = (-0.35 * radius, 0.0, 0.35 * radius)
    origins = []
    for ring_scale in (0.8, 1.2):
        distance = radius * ring_scale
        for slot in range(12):
            angle = math.tau * slot / 12.0
            for z_offset in z_offsets:
                origins.append(region.center + Vector((math.cos(angle) * distance, math.sin(angle) * distance, z_offset)))
    return origins


def _path_origins(region, path_points, radius):
    nearby = sorted(
        (
            point.copy() for point in path_points
            if (Vector((point.x, point.y, 0.0)) - Vector((region.center.x, region.center.y, 0.0))).length
            <= radius * 2.5
        ),
        key=lambda point: ((point - region.center).length, point.x, point.y, point.z),
    )
    if len(nearby) <= 48:
        return nearby
    return [nearby[int(round(index * (len(nearby) - 1) / 47.0))] for index in range(48)]


def _candidate_groups(cache, regions, existing, settings, path_points):
    radius = float(getattr(settings, "patch_candidate_radius", 1.5)) * cache.units_per_meter
    clearance = float(getattr(settings, "patch_camera_safety_distance", 0.25))
    limit_to_path = bool(getattr(settings, "patch_limit_to_path", False)) and path_points
    allow_polar = bool(getattr(settings, "patch_allow_polar", True))
    existing_origins = [candidate.position for candidate in existing]
    duplicate_distance = 0.05 * cache.units_per_meter
    groups = []
    unsafe = 0
    safe_origins = 0
    seen = set()
    origin_tolerance = 0.005 * cache.units_per_meter
    for region_index, region in enumerate(regions):
        origins = _path_origins(region, path_points, radius) if limit_to_path else _ring_origins(cache, region, radius)
        for origin_index, origin in enumerate(origins):
            origin_key = tuple(int(round(value / max(1e-9, origin_tolerance))) for value in origin)
            if origin_key in seen:
                continue
            seen.add(origin_key)
            if any((origin - old).length < duplicate_distance for old in existing_origins):
                continue
            if not scientific_planner._candidate_clear(cache, origin, clearance_m=clearance):
                unsafe += 1
                continue
            safe_origins += 1
            yaw, pitch = _look_angles(origin, region.center)
            offsets = ((0.0, 0.0), (-12.0, 0.0), (12.0, 0.0))
            if allow_polar:
                offsets += ((0.0, -12.0), (0.0, 12.0))
            group = []
            origin_id = f"PATCH_R{region_index:02d}_O{origin_index:04d}"
            for view_index, (yaw_offset, pitch_offset) in enumerate(offsets):
                candidate = scientific_planner.Candidate(
                    candidate_id=f"{origin_id}_V{view_index:02d}",
                    origin_id=origin_id,
                    component_index=region_index,
                    station_index=origin_index,
                    distance_along=0.0,
                    local_step=max(radius * 0.5, clearance * 2.0 * cache.units_per_meter),
                    position=origin.copy(),
                    tangent=(region.center - origin).normalized(),
                    layer_index=0,
                    layer_name=region.name,
                    yaw=(yaw + math.radians(yaw_offset)) % math.tau,
                    pitch=_clamp(pitch + math.radians(pitch_offset), math.radians(-75.0), math.radians(75.0)),
                    kind="patch",
                )
                group.append(candidate)
            groups.append(group)
    return groups, safe_origins, unsafe


def _overlap(candidate, context_candidates):
    cells = scientific_planner._overlap_cell_keys(candidate)
    best_ratio = 0.0
    best_baseline = 0.0
    connected = 0
    if not cells:
        return best_ratio, best_baseline, connected
    for other in context_candidates:
        other_cells = scientific_planner._overlap_cell_keys(other)
        if not other_cells:
            continue
        common = len(cells & other_cells)
        if not common:
            continue
        ratio = common / max(1, min(len(cells), len(other_cells)))
        if ratio >= 0.10:
            connected += 1
        if ratio > best_ratio:
            best_ratio = ratio
            best_baseline = (candidate.position - other.position).length
    return best_ratio, best_baseline, connected


def _candidate_score(cache, candidate, target_keys, coverage, context_candidates, settings):
    maximum_incidence = float(getattr(settings, "scientific_maximum_incidence_angle", 75.0))
    quality = scientific_planner._quality_cell_map(candidate, maximum_incidence)
    hits = target_keys.intersection(quality)
    if not hits:
        return None
    minimum = int(getattr(settings, "patch_min_observation_count", 3))
    recommended = max(minimum, int(getattr(settings, "patch_recommended_observation_count", 5)))
    new_gain = 0
    under_gain = 0.0
    diversity_gain = 0
    for key in hits:
        current = coverage.get(key)
        if current is None:
            new_gain += 1
            under_gain += 1.0
            diversity_gain += 1
            continue
        count = current["observation_count"]
        if count < minimum:
            under_gain += (minimum - count) / max(1, minimum)
        elif count < recommended:
            under_gain += 0.25 * (recommended - count) / max(1, recommended - minimum)
        observation = quality[key]
        if observation.direction_bin not in current["view_direction_bins"]:
            diversity_gain += 1
    best_overlap, baseline, connected = _overlap(candidate, context_candidates)
    preferred_overlap = float(getattr(settings, "patch_min_overlap_ratio", 0.30))
    required_overlap = preferred_overlap if getattr(settings, "patch_prefer_existing_connect", True) else min(0.10, preferred_overlap)
    if best_overlap < required_overlap:
        return None
    target_distance = max(1e-6, (candidate.position - sum((quality[key].position for key in hits), Vector()) / len(hits)).length)
    baseline_ratio = baseline / target_distance
    if baseline_ratio < 0.03:
        parallax = 0.0
    elif baseline_ratio <= 0.35:
        parallax = _clamp(baseline_ratio / 0.20, 0.0, 1.0)
    else:
        parallax = _clamp(1.0 - (baseline_ratio - 0.35) / 1.15, 0.0, 1.0)
    denominator = max(1, len(hits))
    priority = getattr(settings, "patch_priority", "MINIMAL")
    weights = {
        "MINIMAL": (0.38, 0.22, 0.16, 0.14, 0.08, 0.02),
        "COVERAGE": (0.44, 0.26, 0.16, 0.07, 0.05, 0.02),
        "CONNECTIVITY": (0.30, 0.15, 0.12, 0.26, 0.10, 0.07),
    }.get(priority, (0.38, 0.22, 0.16, 0.14, 0.08, 0.02))
    overlap_score = _clamp(best_overlap / max(1e-6, preferred_overlap * 1.5), 0.0, 1.0)
    score = (
        weights[0] * (under_gain / denominator)
        + weights[1] * (new_gain / denominator)
        + weights[2] * (diversity_gain / denominator)
        + weights[3] * overlap_score
        + weights[4] * parallax
        + weights[5] * _clamp(connected / 3.0, 0.0, 1.0)
    )
    return score, under_gain, new_gain, diversity_gain, best_overlap, connected, len(hits)


def _prefilter(cache, groups, target_keys, coverage, existing, settings, progress=None):
    ranked = []
    all_candidates = [candidate for group in groups for candidate in group]
    total = max(1, len(all_candidates))
    for index, candidate in enumerate(all_candidates):
        if progress and (index % 16 == 0 or index + 1 == total):
            progress("局部候选快速预筛", index + 1, total)
        scientific_planner._cast_candidate(cache, candidate, ray_grid=4)
        value = _candidate_score(cache, candidate, target_keys, coverage, existing, settings)
        if value is not None:
            ranked.append((value, candidate))
    maximum = max(24, int(getattr(settings, "patch_max_camera_count", 24)) * 8)
    ranked.sort(key=lambda item: (item[0], item[1].candidate_id), reverse=True)
    kept = []
    per_origin = Counter()
    for value, candidate in ranked:
        if per_origin[candidate.origin_id] >= 2:
            continue
        kept.append(candidate)
        per_origin[candidate.origin_id] += 1
        if len(kept) >= maximum:
            break
    return kept


def _final_cast(cache, candidates, progress=None):
    total = max(1, len(candidates))
    for index, candidate in enumerate(candidates):
        if progress and (index % 8 == 0 or index + 1 == total):
            progress("局部候选完整复检", index + 1, total)
        cache.candidate_rays.pop(candidate.candidate_id, None)
        candidate.quality_maps.clear()
        candidate.overlap_cells.clear()
        scientific_planner._cast_candidate(cache, candidate, ray_grid=cache.ray_grid)


def _select(cache, candidates, target_keys, existing_coverage, existing, settings, progress=None):
    coverage = _copy_coverage(existing_coverage)
    selected = []
    context_candidates = list(existing)
    maximum_incidence = float(getattr(settings, "scientific_maximum_incidence_angle", 75.0))
    minimum = int(getattr(settings, "patch_min_observation_count", 3))
    recommended = max(minimum, int(getattr(settings, "patch_recommended_observation_count", 5)))
    target_ratio = float(getattr(settings, "patch_target_coverage_ratio", 0.95))
    max_count = int(getattr(settings, "patch_max_camera_count", 24))
    remaining = list(candidates)
    used_origins = set()
    while len(selected) < max_count:
        metrics = _measure(target_keys, coverage, minimum, recommended)
        if _satisfied(metrics, target_ratio):
            break
        best = None
        for candidate in remaining:
            if candidate.origin_id in used_origins:
                continue
            value = _candidate_score(cache, candidate, target_keys, coverage, context_candidates, settings)
            if value is None:
                continue
            key = (value, candidate.candidate_id)
            if best is None or key > best[0]:
                best = (key, candidate)
        if best is None:
            break
        candidate = best[1]
        value = best[0][0]
        if value[1] <= 0.0 and value[2] <= 0 and value[3] <= 0:
            break
        selected.append(candidate)
        used_origins.add(candidate.origin_id)
        context_candidates.append(candidate)
        scientific_planner._apply_candidate(coverage, candidate, maximum_incidence)
        if progress:
            progress("增量选择补齐相机", len(selected), max_count)
    return selected, coverage


def plan_patch(scene, settings, existing_cameras, selected_objects=(), path_points=(), progress=None):
    if not existing_cameras:
        raise ValueError("请先生成原始相机，再执行局部样本补齐")
    mode, target_names, object_names, point_filter = _target_spec(scene, settings, selected_objects)
    cache = scientific_planner.make_cache(
        scene,
        settings,
        surface_object_names=object_names,
        surface_point_filter=point_filter,
    )
    if not cache.surface_cells:
        raise ValueError("指定区域内没有可分析的 Mesh 表面")
    existing, existing_coverage = _existing_analysis(cache, existing_cameras, settings, progress)
    minimum = int(getattr(settings, "patch_min_observation_count", 3))
    recommended = max(minimum, int(getattr(settings, "patch_recommended_observation_count", 5)))
    target_ratio = float(getattr(settings, "patch_target_coverage_ratio", 0.95))
    target_keys = set(cache.surface_cells)
    if mode == "AUTO_UNDEROBSERVED":
        under = {
            key for key in target_keys
            if existing_coverage.get(key, {}).get("observation_count", 0) < minimum
        }
        by_object = Counter(cache.surface_cells[key]["object_name"] for key in under)
        selected_names = {name for name, _count in by_object.most_common(8)}
        target_keys = {key for key in under if cache.surface_cells[key]["object_name"] in selected_names}
        target_names = sorted(selected_names)
        if not target_keys:
            target_names = ["auto_underobserved"]
    if not target_keys:
        before = _measure(set(), existing_coverage, minimum, recommended)
        regions = []
    else:
        before = _measure(target_keys, existing_coverage, minimum, recommended)
        regions = _build_regions(mode, target_names, target_keys, cache.surface_cells)
    selected = []
    candidate_count = safe_origins = rejected_unsafe = 0
    after_coverage = _copy_coverage(existing_coverage)
    if target_keys and not _satisfied(before, target_ratio):
        groups, safe_origins, rejected_unsafe = _candidate_groups(
            cache, regions, existing, settings, [Vector(point) for point in path_points]
        )
        candidate_count = sum(len(group) for group in groups)
        kept = _prefilter(cache, groups, target_keys, existing_coverage, existing, settings, progress)
        _final_cast(cache, kept, progress)
        selected, after_coverage = _select(
            cache, kept, target_keys, existing_coverage, existing, settings, progress
        )
    after = _measure(target_keys, after_coverage, minimum, recommended)
    report = {
        "patch_camera_count": len(selected),
        "patch_target_region_count": len(regions),
        "patch_target_surface_cell_count": len(target_keys),
        "patch_under_observed_before": before["under_observed_cell_count"],
        "patch_under_observed_after": after["under_observed_cell_count"],
        "patch_region_coverage_before": before["region_coverage_ratio"],
        "patch_region_coverage_after": after["region_coverage_ratio"],
        "patch_minimum_observation_ratio_before": before["minimum_observation_ratio"],
        "patch_minimum_observation_ratio_after": after["minimum_observation_ratio"],
        "patch_diverse_observation_ratio_before": before["diverse_observation_ratio"],
        "patch_diverse_observation_ratio_after": after["diverse_observation_ratio"],
        "patch_added_camera_names": [],
        "patch_generation_mode": mode,
        "patch_target_names": target_names,
        "patch_candidate_count": candidate_count,
        "patch_safe_origin_count": safe_origins,
        "patch_rejected_unsafe_origin_count": rejected_unsafe,
        "patch_target_coverage_ratio": target_ratio,
        "patch_min_observation_count": minimum,
        "patch_recommended_observation_count": recommended,
        "patch_max_camera_count": int(getattr(settings, "patch_max_camera_count", 24)),
        "patch_min_overlap_ratio": float(getattr(settings, "patch_min_overlap_ratio", 0.30)),
        "patch_camera_safety_distance": float(getattr(settings, "patch_camera_safety_distance", 0.25)),
    }
    return PatchPlan(
        cache, mode, target_names, target_keys, regions, existing, existing_coverage,
        selected, before, after, candidate_count, safe_origins, rejected_unsafe, report,
    )


def create_preview(scene, settings, plan):
    clear_preview(scene)
    collection = _collection(scene, PREVIEW_COLLECTION)
    collection.hide_render = True
    for index, candidate in enumerate(plan.selected, start=1):
        data = bpy.data.cameras.new(f"cam_patch_preview_{index:04d}_Data")
        data.type = "PERSP"
        data.lens = float(settings.focal_length)
        data.clip_start = 0.01
        data.clip_end = float(settings.ray_distance)
        camera = bpy.data.objects.new(f"cam_patch_preview_{index:04d}", data)
        collection.objects.link(camera)
        camera.location = candidate.position
        scientific_planner.orient_camera(camera, candidate.yaw, candidate.pitch)
        camera.hide_render = True
        camera.show_in_front = True
        camera["gs_patch_preview"] = True
        camera["gs_patch_candidate_id"] = candidate.candidate_id
        camera["gs_patch_region"] = candidate.layer_name
    _PREVIEW_PLANS[scene.as_pointer()] = plan
    scene[PREVIEW_REPORT_KEY] = json.dumps(plan.report, ensure_ascii=False)
    return preview_cameras(scene)


def _slug(value):
    value = re.sub(r"[^A-Za-z0-9]+", "_", value).strip("_").lower()
    return value[:32] or "region"


def _load_history(scene):
    try:
        data = json.loads(scene.get(HISTORY_KEY, "[]"))
        return data if isinstance(data, list) else []
    except Exception:
        return []


def apply_preview(scene, settings):
    cameras = preview_cameras(scene)
    if not cameras:
        raise ValueError("没有可应用的补齐预览相机")
    plan = _PREVIEW_PLANS.get(scene.as_pointer())
    final = _collection(scene, FINAL_COLLECTION)
    existing = final_cameras(scene)
    next_index = 1
    for camera in existing:
        match = re.search(r"(\d+)$", camera.name)
        if match:
            next_index = max(next_index, int(match.group(1)) + 1)
    used_stems = {str(camera.get("gs_dataset_image_stem", "")) for camera in existing}
    added_names = []
    added_stems = []
    applied_ids = []
    for offset, camera in enumerate(cameras):
        index = next_index + offset
        region = _slug(str(camera.get("gs_patch_region", "region")))
        name = f"cam_patch_{region}_{index:04d}"
        stem = f"{settings.image_prefix}_patch_{index:04d}"
        while stem in used_stems:
            index += 1
            name = f"cam_patch_{region}_{index:04d}"
            stem = f"{settings.image_prefix}_patch_{index:04d}"
        used_stems.add(stem)
        if final not in camera.users_collection:
            final.objects.link(camera)
        preview_collection = bpy.data.collections.get(PREVIEW_COLLECTION)
        if preview_collection in camera.users_collection:
            preview_collection.objects.unlink(camera)
        camera.name = name
        camera.data.name = name + "_Data"
        camera.hide_render = False
        camera["gs_patch_camera"] = True
        camera["gs_dataset_image_stem"] = stem
        if "gs_patch_preview" in camera:
            del camera["gs_patch_preview"]
        added_names.append(name)
        added_stems.append(stem)
        applied_ids.append(str(camera.get("gs_patch_candidate_id", "")))
    report = dict(plan.report if plan is not None else json.loads(scene.get(PREVIEW_REPORT_KEY, "{}")))
    if plan is not None:
        by_id = {candidate.candidate_id: candidate for candidate in plan.selected}
        coverage = _copy_coverage(plan.existing_coverage)
        maximum_incidence = float(getattr(settings, "scientific_maximum_incidence_angle", 75.0))
        for candidate_id in applied_ids:
            candidate = by_id.get(candidate_id)
            if candidate is not None:
                scientific_planner._apply_candidate(coverage, candidate, maximum_incidence)
        minimum = int(getattr(settings, "patch_min_observation_count", 3))
        recommended = max(minimum, int(getattr(settings, "patch_recommended_observation_count", 5)))
        after = _measure(plan.target_keys, coverage, minimum, recommended)
        report["patch_under_observed_after"] = after["under_observed_cell_count"]
        report["patch_region_coverage_after"] = after["region_coverage_ratio"]
        report["patch_minimum_observation_ratio_after"] = after["minimum_observation_ratio"]
        report["patch_diverse_observation_ratio_after"] = after["diverse_observation_ratio"]
    report["patch_camera_count"] = len(added_names)
    report["patch_added_camera_names"] = added_names
    report["patch_added_image_stems"] = added_stems
    history = _load_history(scene)
    history.append(report)
    scene[HISTORY_KEY] = json.dumps(history, ensure_ascii=False)
    _PREVIEW_PLANS.pop(scene.as_pointer(), None)
    if PREVIEW_REPORT_KEY in scene:
        del scene[PREVIEW_REPORT_KEY]
    return cameras, report


def report_data(scene):
    history = _load_history(scene)
    cameras = final_cameras(scene)
    if not history and not cameras:
        return None
    latest = dict(history[-1]) if history else {}
    if cameras:
        latest["patch_camera_count"] = len(cameras)
        latest["patch_added_camera_names"] = [camera.name for camera in cameras]
        latest["patch_added_image_stems"] = [camera.get("gs_dataset_image_stem", "") for camera in cameras]
    else:
        latest.setdefault("patch_camera_count", 0)
    latest["runs"] = history
    return latest

