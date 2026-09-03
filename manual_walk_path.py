"""Manual viewport-walk recording and collision-cut multi-layer paths.

The base walk curve is immutable input.  Layer settings live on the add-on
property group, while generated valid segments are disposable output consumed
by the existing scientific curve/coverage pipeline.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import bpy
from bpy.types import Operator
from mathutils import Vector

from . import scientific_planner


BASE_COLLECTION = "GS_MANUAL_WALK_BASE"
BASE_OBJECT = "GS_WalkBasePath"
VALID_COLLECTION = "GS_MANUAL_WALK_VALID"
DEBUG_COLLECTION = "GS_MANUAL_WALK_COLLISION_DEBUG"

_RECORD_STATE = {"active": False, "samples": [], "region_data": None, "scene": None}
_BVH_CACHE = {}


def _units_per_meter(scene):
    scale = float(getattr(scene.unit_settings, "scale_length", 1.0) or 1.0)
    return 1.0 / max(1e-9, scale)


def _distance(left, right):
    return (Vector(right) - Vector(left)).length


def polyline_length(points):
    return sum(_distance(left, right) for left, right in zip(points, points[1:]))


def resample_polyline(points, spacing):
    points = [Vector(point) for point in points]
    if len(points) < 2:
        return points
    cumulative = [0.0]
    for left, right in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + (right - left).length)
    total = cumulative[-1]
    if total <= 1e-9:
        return [points[0]]
    count = max(1, int(math.ceil(total / max(spacing, 1e-9))))
    distances = [total * index / count for index in range(count + 1)]
    result = []
    segment = 1
    for distance in distances:
        while segment < len(cumulative) - 1 and cumulative[segment] < distance:
            segment += 1
        span = max(1e-9, cumulative[segment] - cumulative[segment - 1])
        factor = (distance - cumulative[segment - 1]) / span
        result.append(points[segment - 1].lerp(points[segment], factor))
    return result


def process_raw_samples(samples, units_per_meter=1.0):
    """Deduplicate, distance-resample and lightly smooth while retaining stair Z."""
    minimum = 0.02 * units_per_meter
    compact = []
    for point in (Vector(value) for value in samples):
        if not compact or (point - compact[-1]).length >= minimum:
            compact.append(point)
    if len(compact) < 2:
        return compact
    spacing = 0.10 * units_per_meter
    points = resample_polyline(compact, spacing)
    for _pass in range(2):
        candidate = [points[0].copy()]
        for index in range(1, len(points) - 1):
            before, center, after = points[index - 1], points[index], points[index + 1]
            average = (before + after) * 0.5
            xy_factor = 0.18
            transition = max(abs(center.z - before.z), abs(after.z - center.z))
            z_factor = 0.04 if transition >= 0.04 * units_per_meter else 0.12
            candidate.append(Vector((
                center.x + (average.x - center.x) * xy_factor,
                center.y + (average.y - center.y) * xy_factor,
                center.z + (average.z - center.z) * z_factor,
            )))
        candidate.append(points[-1].copy())
        points = candidate
    return resample_polyline(points, spacing)


def default_layer_offsets(layer_count, spacing):
    spacing = float(spacing)
    if int(layer_count) == 2:
        return (spacing * 0.5, -spacing * 0.5)
    if int(layer_count) == 3:
        return (spacing, 0.0, -spacing)
    return (spacing * 1.5, spacing * 0.5, -spacing * 0.5, -spacing * 1.5)


def layer_definitions(settings):
    count = int(getattr(settings, "manual_walk_layer_count", "3"))
    names = {
        2: ("Upper", "Lower"),
        3: ("Upper", "Middle", "Lower"),
        4: ("Upper", "Upper_Middle", "Lower_Middle", "Lower"),
    }[count]
    offsets = [float(getattr(settings, f"manual_walk_layer_{index}_offset")) for index in range(1, 5)]
    return [(index, names[index], offsets[index]) for index in range(count)]


def split_collision_intervals(points, invalid, margin_distance, minimum_length):
    """Expand/merge invalid intervals and return useful valid and debug runs."""
    points = [Vector(point) for point in points]
    if not points:
        return [], []
    expanded = list(bool(value) for value in invalid)
    if any(expanded):
        average_step = polyline_length(points) / max(1, len(points) - 1)
        margin_count = int(math.ceil(margin_distance / max(average_step, 1e-9)))
        original = [index for index, value in enumerate(expanded) if value]
        for index in original:
            for target in range(max(0, index - margin_count), min(len(expanded), index + margin_count + 1)):
                expanded[target] = True

    def runs(wanted):
        result = []
        start = None
        for index, value in enumerate(expanded + [not wanted]):
            matches = (value == wanted) if index < len(expanded) else False
            if matches and start is None:
                start = index
            elif not matches and start is not None:
                result.append(points[start:index])
                start = None
        return result

    valid_runs = [run for run in runs(False) if len(run) >= 2 and polyline_length(run) >= minimum_length]
    invalid_runs = [run for run in runs(True) if len(run) >= 2]
    return valid_runs, invalid_runs


def _remove_collection(name):
    collection = bpy.data.collections.get(name)
    if collection is None:
        return
    for obj in list(collection.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.collections.remove(collection)


def _new_collection(scene, name):
    _remove_collection(name)
    collection = bpy.data.collections.new(name)
    scene.collection.children.link(collection)
    return collection


def _curve_object(collection, name, points, color, **properties):
    if len(points) < 2:
        return None
    data = bpy.data.curves.new(name, "CURVE")
    data.dimensions = "3D"
    data.resolution_u = 1
    data.bevel_depth = 0.008
    spline = data.splines.new("POLY")
    spline.points.add(len(points) - 1)
    for target, point in zip(spline.points, points):
        target.co = (*Vector(point), 1.0)
    obj = bpy.data.objects.new(name, data)
    obj.color = color
    obj.hide_render = True
    obj.show_in_front = True
    for key, value in properties.items():
        obj[key] = value
    collection.objects.link(obj)
    return obj


def _base_points(scene):
    obj = bpy.data.objects.get(BASE_OBJECT)
    if obj is None or obj.type != "CURVE":
        return []
    result = []
    for spline in obj.data.splines:
        if spline.type == "BEZIER":
            result.extend(obj.matrix_world @ point.co for point in spline.bezier_points)
        else:
            result.extend(obj.matrix_world @ point.co.xyz for point in spline.points)
    return result


def create_base_path(scene, points):
    points = [Vector(point) for point in points]
    if len(points) < 2:
        raise RuntimeError("行走样本不足，至少需要两个不同位置。")
    collection = _new_collection(scene, BASE_COLLECTION)
    obj = _curve_object(
        collection, BASE_OBJECT, points, (0.15, 0.55, 1.0, 1.0),
        gs_manual_walk_base=True,
        gs_manual_walk_sample_count=len(points),
    )
    return obj


def _mesh_signature(scene):
    result = []
    for obj in scene.objects:
        if obj.type != "MESH" or obj.get("gs_camera_mesh_visual") or obj.get("gs_scientific_debug"):
            continue
        data_pointer = obj.data.as_pointer() if getattr(obj, "data", None) else 0
        matrix = tuple(round(float(value), 6) for row in obj.matrix_world for value in row)
        result.append((obj.as_pointer(), data_pointer, matrix, len(obj.data.vertices), len(obj.data.polygons)))
    return tuple(result)


def _collision_cache(scene, force=False):
    key = scene.as_pointer()
    signature = _mesh_signature(scene)
    cached = _BVH_CACHE.get(key)
    if not force and cached and cached[0] == signature:
        return cached[1]
    depsgraph = bpy.context.evaluated_depsgraph_get()
    low, high = scientific_planner._scene_bounds(scene)
    cache = SimpleNamespace(
        scene=scene,
        depsgraph=depsgraph,
        scene_diagonal=max(0.01, (high - low).length),
    )
    cache.ray_bvh, cache.ray_faces = scientific_planner._build_scene_bvh(cache)
    _BVH_CACHE[key] = (signature, cache)
    return cache


def _point_collides(cache, point, safety_radius):
    if cache.ray_bvh is None:
        return False
    nearest, _normal, _index, distance = cache.ray_bvh.find_nearest(point, safety_radius)
    if nearest is not None and distance < safety_radius - 1e-6:
        return True
    return scientific_planner._point_inside_solid(cache, Vector(point))


def regenerate_manual_layers(scene, settings, *, force_bvh=False):
    base = _base_points(scene)
    if len(base) < 2:
        return None
    units = _units_per_meter(scene)
    dense_spacing = 0.075 * units
    dense_base = resample_polyline(base, dense_spacing)
    valid_collection = _new_collection(scene, VALID_COLLECTION)
    debug_collection = _new_collection(scene, DEBUG_COLLECTION)
    collision_cut = bool(getattr(settings, "manual_walk_collision_cut", True))
    safety = float(getattr(settings, "manual_walk_safety_radius", 0.20)) * units
    minimum = max(0.50 * units, dense_spacing * 6.0)
    cache = _collision_cache(scene, force=force_bvh) if collision_cut else None
    colors = (
        (0.95, 0.65, 0.10, 1.0),
        (0.20, 0.90, 0.35, 1.0),
        (0.20, 0.75, 0.95, 1.0),
        (0.70, 0.35, 0.95, 1.0),
    )
    summary = {"layers": [], "valid_segments": 0, "invalid_intervals": 0}
    definitions = layer_definitions(settings)
    for layer_index, layer_name, offset_m in definitions:
        offset = offset_m * units
        theoretical = [Vector((point.x, point.y, point.z + offset)) for point in dense_base]
        invalid = (
            [_point_collides(cache, point, safety) for point in theoretical]
            if collision_cut else [False] * len(theoretical)
        )
        valid_runs, invalid_runs = split_collision_intervals(
            theoretical, invalid, safety, minimum
        )
        for segment_index, run in enumerate(valid_runs, 1):
            _curve_object(
                valid_collection,
                f"GS_Manual_{layer_name}_Segment_{segment_index:02d}",
                run, colors[layer_index],
                gs_manual_valid_segment=True,
                gs_manual_layer_name=layer_name,
                gs_manual_layer_index=len(definitions) - 1 - layer_index,
                gs_manual_layer_count=len(definitions),
                gs_manual_z_offset_m=float(offset_m),
                gs_manual_segment_index=segment_index,
            )
        for interval_index, run in enumerate(invalid_runs, 1):
            _curve_object(
                debug_collection,
                f"GS_Manual_{layer_name}_Collision_{interval_index:02d}",
                run, (1.0, 0.05, 0.03, 1.0),
                gs_manual_collision_debug=True,
                gs_manual_layer_name=layer_name,
            )
        summary["layers"].append({
            "name": layer_name,
            "offset_m": float(offset_m),
            "valid_segments": len(valid_runs),
            "invalid_intervals": len(invalid_runs),
        })
        summary["valid_segments"] += len(valid_runs)
        summary["invalid_intervals"] += len(invalid_runs)

    live = bool(getattr(settings, "live_update_cameras", False))
    settings.live_update_cameras = False
    try:
        settings.rig_mode = "PATH"
        settings.path_capture_mode = "SCIENTIFIC_THREE_LAYER"
        settings.scientific_origin_mode = "MANUAL_CURVE"
        settings.scientific_realization_mode = "SCIENTIFIC_POSE_SEQUENCE"
        settings.path_object = None
        settings.path_collection = valid_collection
    finally:
        settings.live_update_cameras = live
    settings.manual_walk_status = (
        f"已生成 {summary['valid_segments']} 条安全路径，裁掉 {summary['invalid_intervals']} 个碰撞区间"
    )
    scene["gs_manual_walk_summary"] = str(summary)
    return summary


def clear_manual_paths(scene, settings):
    generated = bpy.data.collections.get(VALID_COLLECTION)
    if getattr(settings, "path_collection", None) is generated:
        settings.path_collection = None
    for name in (BASE_COLLECTION, VALID_COLLECTION, DEBUG_COLLECTION):
        _remove_collection(name)
    _RECORD_STATE.update(active=False, samples=[], region_data=None, scene=None)
    settings.manual_walk_recording = False
    settings.manual_walk_status = "尚未录制行走路径"


def _viewport_context(context):
    window = context.window
    screen = window.screen if window else None
    if not screen:
        return None
    preferred = context.area if context.area and context.area.type == "VIEW_3D" else None
    areas = ([preferred] if preferred else []) + [area for area in screen.areas if area.type == "VIEW_3D" and area is not preferred]
    for area in areas:
        region = next((item for item in area.regions if item.type == "WINDOW"), None)
        space = area.spaces.active
        if region and getattr(space, "region_3d", None):
            return window, area, region, space.region_3d
    return None


def _record_tick():
    if not _RECORD_STATE["active"]:
        return None
    region_data = _RECORD_STATE.get("region_data")
    scene = _RECORD_STATE.get("scene")
    if region_data is None or scene is None:
        return None
    try:
        point = region_data.view_matrix.inverted().translation.copy()
        samples = _RECORD_STATE["samples"]
        threshold = 0.015 * _units_per_meter(scene)
        if not samples or (point - samples[-1]).length >= threshold:
            samples.append(point)
        settings = scene.gs_colmap_settings
        settings.manual_walk_status = f"正在录制：{len(samples)} 个原始样本"
    except Exception:
        pass
    return 0.05


def start_recording(context):
    if _RECORD_STATE["active"]:
        raise RuntimeError("行走路径已经在录制。")
    viewport = _viewport_context(context)
    if viewport is None:
        raise RuntimeError("请在 3D 视图中开始行走录制。")
    window, area, region, region_data = viewport
    scene = context.scene
    _RECORD_STATE.update(active=True, samples=[], region_data=region_data, scene=scene)
    scene.gs_colmap_settings.manual_walk_recording = True
    scene.gs_colmap_settings.manual_walk_status = "正在启动 Blender Walk Navigation"
    if not bpy.app.timers.is_registered(_record_tick):
        bpy.app.timers.register(_record_tick, first_interval=0.01)
    try:
        with context.temp_override(window=window, area=area, region=region):
            bpy.ops.view3d.walk("INVOKE_DEFAULT")
    except Exception:
        _RECORD_STATE["active"] = False
        scene.gs_colmap_settings.manual_walk_recording = False
        raise


def stop_recording(scene, settings):
    if not _RECORD_STATE["active"]:
        raise RuntimeError("当前没有正在录制的行走路径。")
    _record_tick()
    samples = list(_RECORD_STATE["samples"])
    _RECORD_STATE.update(active=False, samples=[], region_data=None, scene=None)
    settings.manual_walk_recording = False
    points = process_raw_samples(samples, _units_per_meter(scene))
    create_base_path(scene, points)
    return regenerate_manual_layers(scene, settings, force_bvh=True)


class GSCOLMAP_OT_manual_walk_start(Operator):
    bl_idname = "gs_colmap.manual_walk_start"
    bl_label = "开始录制行走路径"
    bl_description = "启动 Blender Walk Navigation，并持续记录视口世界坐标 XYZ"

    def execute(self, context):
        try:
            start_recording(context)
        except Exception as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        self.report({"INFO"}, "Walk Navigation 已启动；完成行走后确认导航，再点击结束录制")
        return {"FINISHED"}


class GSCOLMAP_OT_manual_walk_stop(Operator):
    bl_idname = "gs_colmap.manual_walk_stop"
    bl_label = "结束录制"
    bl_description = "结束采样，生成 BasePath、多层安全路径并自动裁掉碰撞区间"

    def execute(self, context):
        try:
            summary = stop_recording(context.scene, context.scene.gs_colmap_settings)
        except Exception as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        self.report({"INFO"}, f"行走路径完成：{summary['valid_segments']} 条有效 Segment")
        return {"FINISHED"}


class GSCOLMAP_OT_manual_walk_regenerate(Operator):
    bl_idname = "gs_colmap.manual_walk_regenerate"
    bl_label = "重新生成多层路径"
    bl_description = "从未修改的 BasePath 重新偏移并执行整条样条密集碰撞裁切"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        try:
            summary = regenerate_manual_layers(
                context.scene, context.scene.gs_colmap_settings, force_bvh=True
            )
            if summary is None:
                raise RuntimeError("请先录制 Base Walk Path。")
        except Exception as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        self.report({"INFO"}, f"已重新生成 {summary['valid_segments']} 条安全路径")
        return {"FINISHED"}


class GSCOLMAP_OT_manual_walk_clear(Operator):
    bl_idname = "gs_colmap.manual_walk_clear"
    bl_label = "清除路径"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        clear_manual_paths(context.scene, context.scene.gs_colmap_settings)
        return {"FINISHED"}


CLASSES = (
    GSCOLMAP_OT_manual_walk_start,
    GSCOLMAP_OT_manual_walk_stop,
    GSCOLMAP_OT_manual_walk_regenerate,
    GSCOLMAP_OT_manual_walk_clear,
)
