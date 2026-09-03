"""Deterministic indoor camera planning for scientific three-layer capture.

This module is intentionally limited to path planning. Rendering and export stay
in the add-on's main module so the legacy dataset contract remains untouched.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

from . import free_space_planner


BASE_OFFSETS = (-0.40, -0.24, -0.08, 0.08, 0.24, 0.40)
BASE_YAWS = (0.0, 60.0, 120.0, 180.0, 240.0, 300.0)
BASE_PITCHES = (20.0, 0.0, -20.0, 20.0, 0.0, -20.0)
DEBUG_COLLECTION = "GS_SCIENTIFIC_PATH_DEBUG"


class PlanningCancelled(RuntimeError):
    pass


@dataclass
class RayObservation:
    key: tuple
    object_name: str
    face_index: int
    position: Vector
    normal: Vector
    distance: float
    view_direction: Vector
    incidence_angle: float
    category: str
    direction_bin: tuple
    candidate_id: str
    footprint_radius: int


@dataclass
class Candidate:
    candidate_id: str
    origin_id: str
    component_index: int
    station_index: int
    distance_along: float
    local_step: float
    position: Vector
    tangent: Vector
    layer_index: int
    layer_name: str
    yaw: float
    pitch: float
    provider_type: str = "CURVE"
    region_id: str = ""
    kind: str = "normal"
    critical: bool = False
    is_base: bool = False
    distance_band: str = "REGULAR"
    view_role: str = "general"
    nearest_target_distance: float = 0.0
    reference_target_distance: float = 0.0
    dominant_near_surface_ratio: float = 0.0
    dominant_near_surface_key: tuple = None
    shared_environment_ratio: float = 1.0
    near_field_rejected: bool = False
    near_field_rejection_reason: str = ""
    strong_mid_overlap_count: int = 0
    near_field_average_overlap: float = 0.0
    observations: dict = field(default_factory=dict)
    score: float = 0.0
    overlap_cells: dict = field(default_factory=dict)
    ray_grid: int = 0
    quality_maps: dict = field(default_factory=dict)


@dataclass
class PlanningCache:
    scene: object
    settings: object
    depsgraph: object
    scene_min: Vector
    scene_max: Vector
    scene_diagonal: float
    units_per_meter: float
    horizontal_fov: float
    vertical_fov: float
    coverage_cell_size: float
    ray_grid: int
    prefilter_ray_grid: int
    candidate_rays: dict = field(default_factory=dict)
    vertical_probes: dict = field(default_factory=dict)
    mesh_world_matrices: dict = field(default_factory=dict)
    surface_cells: dict = field(default_factory=dict)
    ray_bvh: object = None
    ray_faces: list = field(default_factory=list)
    origin_generation: dict = field(default_factory=dict)
    free_space_map: object = None
    coverage_driven_reserve: int = 0
    rejected_too_close_candidate_ids: set = field(default_factory=set)
    dominant_surface_rejected_ids: set = field(default_factory=set)
    near_field_rejected_positions: dict = field(default_factory=dict)
    near_field_candidate_ids: set = field(default_factory=set)
    near_field_quota_rejected_ids: set = field(default_factory=set)


@dataclass
class OriginSeed:
    position: Vector
    preferred_direction: Vector
    local_clearance: float
    floor_z: float
    ceiling_z: float
    layer_name: str
    region_id: str
    provider_type: str
    is_critical: bool
    source_reference: str
    origin_id: str
    component_index: int
    station_index: int
    distance_along: float
    local_step: float
    tangent: Vector
    layer_index: int
    base_yaw: float
    base_pitch: float
    polar_yaw: float
    allowed_pitches: tuple
    allow_polar: bool = False
    distance_band: str = "REGULAR"
    view_role: str = "general"


class OriginProvider:
    provider_type = "BASE"

    def prepare(self, context, planning_cache, settings):
        self.context = context
        self.cache = planning_cache
        self.settings = settings
        return self

    def generate_origins(self):
        raise NotImplementedError


def _clamp(value, low, high):
    return max(low, min(high, value))


def _unit_scale(scene):
    scale = float(getattr(scene.unit_settings, "scale_length", 1.0) or 1.0)
    return 1.0 / max(1e-9, scale)


def _scene_bounds(scene):
    low = Vector((1e18, 1e18, 1e18))
    high = Vector((-1e18, -1e18, -1e18))
    found = False
    for obj in scene.objects:
        if obj.type != "MESH" or obj.get("gs_camera_mesh_visual") or obj.get("gs_scientific_debug"):
            continue
        try:
            if not obj.visible_get():
                continue
        except Exception:
            pass
        for corner in obj.bound_box:
            point = obj.matrix_world @ Vector(corner)
            low.x, low.y, low.z = min(low.x, point.x), min(low.y, point.y), min(low.z, point.z)
            high.x, high.y, high.z = max(high.x, point.x), max(high.y, point.y), max(high.z, point.z)
            found = True
    if not found:
        low = Vector((-5.0, -5.0, -2.5))
        high = Vector((5.0, 5.0, 2.5))
    return low, high


def _camera_fov(scene, settings):
    lens = max(1e-6, float(settings.focal_length))
    sensor_width = 36.0
    horizontal = 2.0 * math.atan(sensor_width / (2.0 * lens))
    width = max(1.0, float(settings.resolution_x))
    height = max(1.0, float(settings.resolution_y))
    vertical = 2.0 * math.atan(math.tan(horizontal * 0.5) * height / width)
    return horizontal, vertical


def make_cache(scene, settings, surface_object_names=None, surface_point_filter=None):
    low, high = _scene_bounds(scene)
    diagonal = max(0.01, (high - low).length)
    units_per_meter = _unit_scale(scene)
    hfov, vfov = _camera_fov(scene, settings)
    cell_m = _clamp((diagonal / units_per_meter) / 300.0, 0.03, 0.15)
    quality = getattr(settings, "scientific_ray_quality", "NORMAL")
    grid = {"FAST": 8, "NORMAL": 12, "HIGH": 16}.get(quality, 12)
    cache = PlanningCache(
        scene=scene,
        settings=settings,
        depsgraph=bpy.context.evaluated_depsgraph_get(),
        scene_min=low,
        scene_max=high,
        scene_diagonal=diagonal,
        units_per_meter=units_per_meter,
        horizontal_fov=hfov,
        vertical_fov=vfov,
        coverage_cell_size=cell_m * units_per_meter,
        ray_grid=grid,
        prefilter_ray_grid=4,
    )
    cache.mesh_world_matrices = {
        obj.name_full: obj.matrix_world.copy()
        for obj in scene.objects
        if obj.type == "MESH" and not obj.get("gs_camera_mesh_visual") and not obj.get("gs_scientific_debug")
    }
    cache.ray_bvh, cache.ray_faces = _build_scene_bvh(cache)
    cache.surface_cells = _estimate_surface_cells(
        cache, object_names=surface_object_names, point_filter=surface_point_filter
    )
    return cache


def _build_scene_bvh(cache):
    vertices = []
    polygons = []
    faces = []
    for obj in cache.scene.objects:
        if obj.type != "MESH" or obj.get("gs_camera_mesh_visual") or obj.get("gs_scientific_debug"):
            continue
        try:
            if not obj.visible_get():
                continue
        except Exception:
            pass
        evaluated = obj.evaluated_get(cache.depsgraph)
        mesh = evaluated.to_mesh()
        matrix = evaluated.matrix_world.copy()
        vertex_offset = len(vertices)
        try:
            vertices.extend(matrix @ vertex.co for vertex in mesh.vertices)
            for polygon in mesh.polygons:
                if len(polygon.vertices) < 3:
                    continue
                polygons.append(tuple(vertex_offset + index for index in polygon.vertices))
                faces.append((obj, int(polygon.index)))
        finally:
            evaluated.to_mesh_clear()
    if not polygons:
        return None, []
    return BVHTree.FromPolygons(vertices, polygons, all_triangles=False), faces


def _ray_cast(cache, origin, direction, distance):
    if cache.ray_bvh is not None:
        location, normal, index, hit_distance = cache.ray_bvh.ray_cast(
            origin, direction, distance
        )
        if location is None or index is None:
            return False, None, None, -1, None
        obj, face_index = cache.ray_faces[index]
        return True, location, normal, face_index, obj
    hit, location, normal, face_index, obj, _matrix = cache.scene.ray_cast(
        cache.depsgraph, origin, direction, distance=distance
    )
    return hit, location, normal, face_index, obj


def _polyline_metrics(points):
    cumulative = [0.0]
    for start, end in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + (end - start).length)
    return cumulative


def _point_at(points, cumulative, distance):
    if not points:
        return Vector(), Vector((1.0, 0.0, 0.0))
    if len(points) == 1 or cumulative[-1] <= 1e-9:
        return points[0].copy(), Vector((1.0, 0.0, 0.0))
    distance = _clamp(distance, 0.0, cumulative[-1])
    segment = 1
    while segment < len(cumulative) - 1 and cumulative[segment] < distance:
        segment += 1
    start = points[segment - 1]
    end = points[segment]
    span = max(1e-9, cumulative[segment] - cumulative[segment - 1])
    factor = (distance - cumulative[segment - 1]) / span
    tangent = end - start
    if tangent.length <= 1e-9:
        tangent = Vector((1.0, 0.0, 0.0))
    return start.lerp(end, factor), tangent.normalized()


def _percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    position = _clamp(fraction, 0.0, 1.0) * (len(values) - 1)
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return values[low]
    return values[low] + (values[high] - values[low]) * (position - low)


def _reference_depth(cache, point, tangent):
    base = math.atan2(tangent.y, tangent.x)
    distances = []
    search = cache.scene_diagonal * 1.25
    for index in range(16):
        yaw = base + math.tau * index / 16.0
        direction = Vector((math.cos(yaw), math.sin(yaw), 0.0))
        hit, location, _normal, _face, _obj = _ray_cast(
            cache, point + direction * 1e-4, direction, search
        )
        if hit:
            distance = (location - point).length
            if distance > 1e-4:
                distances.append(distance)
    return _percentile(distances, 0.40) or max(cache.units_per_meter, search * 0.25)


def _local_step(cache, point, tangent):
    settings = cache.settings
    min_step = float(settings.scientific_minimum_step) * cache.units_per_meter
    max_step = max(min_step, float(settings.scientific_maximum_step) * cache.units_per_meter)
    depth = _reference_depth(cache, point, tangent)
    raw = 2.0 * depth * math.tan(cache.horizontal_fov * 0.5) * (1.0 - settings.scientific_target_overlap)
    return _clamp(raw, min_step, max_step), depth


def _heading_key_distances(points, cumulative, max_change_radians):
    keys = {0.0, cumulative[-1] if cumulative else 0.0}
    for index in range(1, len(points) - 1):
        before = points[index] - points[index - 1]
        after = points[index + 1] - points[index]
        if before.length <= 1e-9 or after.length <= 1e-9:
            continue
        before_normalized = before.normalized()
        after_normalized = after.normalized()
        heading_change = before.angle(after) >= max_change_radians
        before_grade = abs(float(before_normalized.z))
        after_grade = abs(float(after_normalized.z))
        connector_transition = (
            (before_grade < 0.06 <= after_grade)
            or (after_grade < 0.06 <= before_grade)
            or abs(before_grade - after_grade) >= 0.10
        )
        if heading_change or connector_transition:
            keys.add(cumulative[index])
    return keys


def _dedupe_distances(distances, tolerance=1e-5):
    result = []
    for value in sorted(distances):
        if not result or abs(value - result[-1]) > tolerance:
            result.append(value)
    return result


def _curvature_spacing_factor(points, cumulative, distance, base_step):
    """Keep FOV/overlap spacing authoritative, then densify bends and stairs."""
    total = cumulative[-1] if cumulative else 0.0
    probe = max(base_step * 0.35, total * 1e-5, 1e-6)
    before_distance = max(0.0, distance - probe)
    after_distance = min(total, distance + probe)
    _before_point, before_tangent = _point_at(points, cumulative, before_distance)
    _point, tangent = _point_at(points, cumulative, distance)
    _after_point, after_tangent = _point_at(points, cumulative, after_distance)
    span = max(1e-6, after_distance - before_distance)
    curvature = before_tangent.angle(after_tangent) / span
    stair_grade = abs(float(tangent.z))
    return 1.0 / (1.0 + 0.75 * curvature * base_step + 0.45 * stair_grade)


def _adaptive_stations(cache, component, legacy_count):
    points = component["points"]
    cumulative = _polyline_metrics(points)
    total = cumulative[-1] if cumulative else 0.0
    if total <= 1e-9:
        point = points[0].copy()
        step, depth = _local_step(cache, point, Vector((1.0, 0.0, 0.0)))
        return [{"distance": 0.0, "point": point, "tangent": Vector((1.0, 0.0, 0.0)), "step": step, "depth": depth, "critical": True}]

    critical_distances = _heading_key_distances(
        points, cumulative, math.radians(float(cache.settings.scientific_maximum_heading_change))
    )
    if total > 1e-6:
        critical_distances.add(total * 0.5)
    distances = set(critical_distances)
    for key_distance in critical_distances:
        if key_distance <= 1e-6 or key_distance >= total - 1e-6:
            continue
        key_point, key_tangent = _point_at(points, cumulative, key_distance)
        key_step, _depth = _local_step(cache, key_point, key_tangent)
        distances.update((_clamp(key_distance - key_step * 0.5, 0.0, total), _clamp(key_distance + key_step * 0.5, 0.0, total)))
    legacy_count = max(1, int(legacy_count))
    for index in range(legacy_count):
        distances.add(total * index / max(1, legacy_count - 1))

    cursor = 0.0
    guard = 0
    while cursor < total - 1e-6 and guard < 100000:
        point, tangent = _point_at(points, cumulative, cursor)
        step, _depth = _local_step(cache, point, tangent)
        step *= _curvature_spacing_factor(points, cumulative, cursor, step)
        step = max(float(cache.settings.scientific_minimum_step) * cache.units_per_meter * 0.50, step)
        cursor = min(total, cursor + step)
        distances.add(cursor)
        guard += 1

    stations = []
    for distance in _dedupe_distances(distances):
        point, tangent = _point_at(points, cumulative, distance)
        step, depth = _local_step(cache, point, tangent)
        critical = any(abs(distance - key) <= 1e-5 for key in critical_distances)
        stations.append({
            "distance": distance,
            "point": point,
            "tangent": tangent,
            "step": step,
            "depth": depth,
            "critical": critical,
        })
    return stations


def _vertical_limits(cache, point):
    key = (round(point.x, 4), round(point.y, 4), round(point.z, 4))
    cached = cache.vertical_probes.get(key)
    if cached is not None:
        return cached
    distance = cache.scene_diagonal * 1.5
    up = Vector((0.0, 0.0, 1.0))
    down = -up
    floor_hit, floor_location, *_ = _ray_cast(cache, point + up * 1e-4, down, distance)
    ceil_hit, ceil_location, *_ = _ray_cast(cache, point + down * 1e-4, up, distance)
    if floor_hit and ceil_hit and ceil_location.z > floor_location.z:
        result = (floor_location.z, ceil_location.z)
    else:
        half = 1.2 * cache.units_per_meter
        result = (point.z - half, point.z + half)
    cache.vertical_probes[key] = result
    return result


def _layer_z_values(cache, point, layer_count):
    floor_z, ceiling_z = _vertical_limits(cache, point)
    height = max(1e-6, ceiling_z - floor_z)
    clearance = _clamp(0.12 * height, 0.30 * cache.units_per_meter, 0.45 * cache.units_per_meter)
    clearance = min(clearance, max(0.05 * cache.units_per_meter, height * 0.24))
    low = floor_z + clearance
    high = ceiling_z - clearance
    if layer_count <= 1 or high <= low:
        return [(floor_z + ceiling_z) * 0.5]
    return [low + (high - low) * index / (layer_count - 1) for index in range(layer_count)]


def _layer_names(count):
    if count == 2:
        return ("Low", "High")
    if count == 3:
        return ("Low", "Middle", "High")
    return tuple("Low" if i == 0 else "High" if i == count - 1 else f"Middle_{i}" for i in range(count))


def _base_layer_indices(count):
    if count == 2:
        return (0, 1, 1, 0, 1, 0)
    if count == 3:
        return (0, 1, 2, 0, 1, 2)
    return (0, 1, count - 1, 0, count - 2, count - 1)


def _allowed_pitches(layer_index, layer_count):
    if layer_index == 0:
        return (10.0, 25.0)
    if layer_index == layer_count - 1:
        return (-25.0, -10.0)
    return (-10.0, 0.0, 10.0)


def _origin_key(position, tolerance):
    tolerance = max(1e-9, tolerance)
    return tuple(int(round(value / tolerance)) for value in position)


def _candidate_direction(yaw, pitch):
    cp = math.cos(pitch)
    return Vector((cp * math.cos(yaw), cp * math.sin(yaw), math.sin(pitch))).normalized()


def _direction_basis(yaw, pitch):
    forward = _candidate_direction(yaw, pitch)
    world_up = Vector((0.0, 0.0, 1.0))
    right = forward.cross(world_up)
    if right.length <= 1e-8:
        right = Vector((1.0, 0.0, 0.0))
    right.normalize()
    up = right.cross(forward).normalized()
    return forward, right, up


def _surface_category(normal):
    nz = normal.normalized().z
    if nz >= math.cos(math.radians(25.0)):
        return "floor_like"
    if nz <= -math.cos(math.radians(25.0)):
        return "ceiling_like"
    if abs(nz) <= math.sin(math.radians(25.0)):
        return "vertical_like"
    if abs(nz) < 0.95:
        return "slanted"
    return "other"


def _surface_key(cache, obj, face_index, position, normal):
    cell = cache.coverage_cell_size
    qpos = tuple(int(math.floor(value / cell + 0.5)) for value in position)
    qnormal = tuple(int(round(value * 8.0)) for value in normal.normalized())
    return (obj.name_full if obj else "<none>", int(face_index), qpos, qnormal)


def _expanded_surface_map(observations, radius=None):
    """Fill the tangent-plane footprint represented by a low-resolution ray hit."""
    expanded = {}
    for key, observation in observations.items():
        object_name, face_index, position_bin, normal_bin = key
        tangent_axes = sorted(range(3), key=lambda axis: abs(normal_bin[axis]))[:2]
        first_axis, second_axis = tangent_axes
        footprint_radius = observation.footprint_radius if radius is None else radius
        for first in range(-footprint_radius, footprint_radius + 1):
            for second in range(-footprint_radius, footprint_radius + 1):
                shifted = list(position_bin)
                shifted[first_axis] += first
                shifted[second_axis] += second
                expanded_key = (object_name, face_index, tuple(shifted), normal_bin)
                old = expanded.get(expanded_key)
                if old is None or observation.incidence_angle < old.incidence_angle:
                    expanded[expanded_key] = observation
    return expanded

def _estimate_surface_cells(
    cache, maximum_cells=200000, object_names=None, point_filter=None
):
    """Conservatively sample evaluated mesh faces into position-aware cells."""
    cells = {}
    cell = max(1e-6, cache.coverage_cell_size)
    for obj in cache.scene.objects:
        if obj.type != "MESH" or obj.get("gs_camera_mesh_visual") or obj.get("gs_scientific_debug"):
            continue
        if object_names is not None and obj.name_full not in object_names:
            continue
        evaluated = obj.evaluated_get(cache.depsgraph)
        mesh = evaluated.to_mesh()
        matrix = evaluated.matrix_world.copy()
        normal_matrix = matrix.to_3x3().inverted().transposed()
        try:
            for polygon in mesh.polygons:
                if len(cells) >= maximum_cells or len(polygon.vertices) < 3:
                    break
                world_vertices = [matrix @ mesh.vertices[index].co for index in polygon.vertices]
                normal = (normal_matrix @ polygon.normal).normalized()
                anchor = world_vertices[0]
                for triangle_index in range(1, len(world_vertices) - 1):
                    a, b, c = anchor, world_vertices[triangle_index], world_vertices[triangle_index + 1]
                    longest = max((b - a).length, (c - a).length, (c - b).length)
                    subdivisions = max(1, min(64, int(math.ceil(longest / cell))))
                    # Cap very large faces while retaining samples across their full area.
                    subdivisions = min(subdivisions, max(1, int(math.sqrt(4096))))
                    for row in range(subdivisions + 1):
                        for column in range(subdivisions + 1 - row):
                            if len(cells) >= maximum_cells:
                                break
                            u = row / subdivisions
                            v = column / subdivisions
                            position = a + (b - a) * u + (c - a) * v
                            if point_filter is not None and not point_filter(obj, position):
                                continue
                            key = _surface_key(cache, obj, polygon.index, position, normal)
                            cells[key] = {
                                "category": _surface_category(normal),
                                "position": position.copy(),
                                "object_name": obj.name_full,
                            }
                        if len(cells) >= maximum_cells:
                            break
                    if len(cells) >= maximum_cells:
                        break
        finally:
            evaluated.to_mesh_clear()
        if len(cells) >= maximum_cells:
            break
    return cells

def _direction_bin(direction):
    yaw = math.atan2(direction.y, direction.x) % math.tau
    pitch = math.asin(_clamp(direction.normalized().z, -1.0, 1.0))
    size = math.radians(15.0)
    return (int(round(yaw / size)), int(round(pitch / size)))


def _update_near_field_metrics(
    cache, candidate, ray_distances, near_surface_counts, total_ray_count,
):
    candidate.near_field_rejected = False
    candidate.near_field_rejection_reason = ""
    candidate.strong_mid_overlap_count = 0
    candidate.near_field_average_overlap = 0.0
    cache.near_field_candidate_ids.discard(candidate.candidate_id)
    if not ray_distances:
        candidate.nearest_target_distance = 0.0
        candidate.reference_target_distance = 0.0
        candidate.dominant_near_surface_ratio = 0.0
        candidate.dominant_near_surface_key = None
        candidate.shared_environment_ratio = 0.0
        return
    units = max(1e-9, cache.units_per_meter)
    candidate.nearest_target_distance = min(ray_distances) / units
    candidate.reference_target_distance = (_percentile(ray_distances, 0.25) or min(ray_distances)) / units
    if near_surface_counts:
        dominant_key, dominant_count = max(
            near_surface_counts.items(),
            key=lambda item: (item[1], item[0]),
        )
        candidate.dominant_near_surface_key = dominant_key
        candidate.dominant_near_surface_ratio = dominant_count / max(1, total_ray_count)
    else:
        candidate.dominant_near_surface_key = None
        candidate.dominant_near_surface_ratio = 0.0
    candidate.shared_environment_ratio = 0.0
    if candidate.provider_type != "CURVE":
        recommended_min = float(getattr(cache.settings, "near_field_recommended_distance_min", 0.60))
        recommended_max = max(
            recommended_min,
            float(getattr(cache.settings, "near_field_recommended_distance_max", 1.00)),
        )
        if candidate.reference_target_distance < recommended_min:
            candidate.distance_band = "NEAR"
            cache.near_field_candidate_ids.add(candidate.candidate_id)
        elif candidate.reference_target_distance <= recommended_max * 1.25:
            candidate.distance_band = "MID"
        else:
            candidate.distance_band = "FAR"
        step_ratio = _clamp(
            float(getattr(cache.settings, "near_field_step_distance_ratio", 0.25)),
            0.20, 0.30,
        )
        minimum_spacing = float(
            getattr(cache.settings, "near_field_minimum_origin_spacing", 0.12)
        ) * units
        distance_limited_step = candidate.nearest_target_distance * step_ratio * units
        candidate.local_step = max(
            minimum_spacing,
            min(candidate.local_step, distance_limited_step),
        )
        unsuitable = float(getattr(cache.settings, "near_field_unsuitable_distance", 0.35))
        if candidate.reference_target_distance < unsuitable and candidate.view_role != "door_detail":
            cache.rejected_too_close_candidate_ids.add(candidate.candidate_id)
            cache.near_field_rejected_positions[candidate.candidate_id] = candidate.position.copy()


def _cast_candidate(cache, candidate, progress=None, cancel=None, ray_grid=None):
    if candidate.candidate_id in cache.candidate_rays:
        candidate.observations = cache.candidate_rays[candidate.candidate_id]
        return
    if cancel and cancel():
        raise PlanningCancelled("Scientific camera planning cancelled")
    forward, right, up = _direction_basis(candidate.yaw, candidate.pitch)
    tan_h = math.tan(cache.horizontal_fov * 0.5)
    tan_v = math.tan(cache.vertical_fov * 0.5)
    candidate.quality_maps.clear()
    observations = {}
    sample_grid = int(ray_grid or cache.ray_grid)
    projection_grid = max(sample_grid, cache.ray_grid)
    if sample_grid == 4 and projection_grid > sample_grid:
        first = max(0, min(projection_grid - 1, int(round(projection_grid * 0.125 - 0.5))))
        second = max(first + 1, min(projection_grid - 1, int(round(projection_grid * 0.375 - 0.5))))
        sample_indices = (first, second, projection_grid - 1 - second, projection_grid - 1 - first)
    else:
        sample_indices = range(sample_grid)
        projection_grid = sample_grid
    ray_distances = []
    near_surface_counts = {}
    near_limit = float(getattr(cache.settings, "near_field_recommended_distance_min", 0.60)) * cache.units_per_meter
    for row in sample_indices:
        v = (2.0 * (row + 0.5) / projection_grid - 1.0) * tan_v
        for column in sample_indices:
            u = (2.0 * (column + 0.5) / projection_grid - 1.0) * tan_h
            direction = (forward + right * u + up * v).normalized()
            hit, location, normal, face, obj = _ray_cast(
                cache,
                candidate.position + direction * 1e-4,
                direction,
                cache.scene_diagonal * 1.5,
            )
            if not hit or obj is None:
                continue
            distance = (location - candidate.position).length
            ray_distances.append(distance)
            if distance < near_limit:
                object_key = ("OBJECT", obj.name_full)
                surface_key = (
                    "SURFACE", obj.name_full,
                    int(round(normal.x * 8.0)),
                    int(round(normal.y * 8.0)),
                    int(round(normal.z * 8.0)),
                )
                near_surface_counts[object_key] = near_surface_counts.get(object_key, 0) + 1
                near_surface_counts[surface_key] = near_surface_counts.get(surface_key, 0) + 1
            ray_footprint = distance * max(2.0 * tan_h / projection_grid, 2.0 * tan_v / projection_grid)
            footprint_radius = int(_clamp(math.ceil(ray_footprint / (2.0 * cache.coverage_cell_size)), 1, 6))
            incidence = math.degrees(math.acos(_clamp(abs(normal.normalized().dot(-direction)), 0.0, 1.0)))
            key = _surface_key(cache, obj, face, location, normal)
            observation = RayObservation(
                key=key,
                object_name=obj.name_full,
                face_index=int(face),
                position=location.copy(),
                normal=normal.copy(),
                distance=distance,
                view_direction=(-direction).copy(),
                incidence_angle=incidence,
                category=_surface_category(normal),
                direction_bin=_direction_bin(-direction),
                candidate_id=candidate.candidate_id,
                footprint_radius=footprint_radius,
            )
            old = observations.get(key)
            if old is None or observation.incidence_angle < old.incidence_angle:
                observations[key] = observation
    candidate.observations = observations
    _update_near_field_metrics(
        cache,
        candidate,
        ray_distances,
        near_surface_counts,
        len(sample_indices) * len(sample_indices),
    )
    candidate.ray_grid = sample_grid
    cache.candidate_rays[candidate.candidate_id] = observations


def _point_inside_solid(cache, position):
    directions = (
        Vector((1.0, 0.371, 0.197)).normalized(),
        Vector((-0.283, 1.0, 0.419)).normalized(),
        Vector((0.347, -0.229, 1.0)).normalized(),
    )
    epsilon = max(1e-5, cache.scene_diagonal * 1e-7)
    inside_votes = 0
    for direction in directions:
        origin = position + direction * epsilon
        remaining = cache.scene_diagonal * 2.0
        intersections = 0
        for _intersection in range(64):
            hit, location, *_ = _ray_cast(cache, origin, direction, remaining)
            if not hit:
                break
            travelled = (location - origin).length
            intersections += 1
            step = travelled + epsilon
            remaining -= step
            if remaining <= epsilon:
                break
            origin = location + direction * epsilon
        if intersections % 2:
            inside_votes += 1
    return inside_votes >= 2


def _candidate_clear(cache, position, clearance_m=None):
    if _point_inside_solid(cache, position):
        return False
    clearance = float(
        cache.settings.scientific_camera_clearance if clearance_m is None else clearance_m
    ) * cache.units_per_meter
    if cache.ray_bvh is not None:
        nearest, _normal, _index, distance = cache.ray_bvh.find_nearest(position, clearance)
        return nearest is None or distance >= clearance - 1e-5

    # Fallback for scenes where a combined BVH could not be built. The normal path uses
    # the exact nearest-surface query above, which is both faster and safer than sampling.
    golden_angle = math.pi * (3.0 - math.sqrt(5.0))
    for index in range(60):
        z = 1.0 - 2.0 * (index + 0.5) / 60.0
        radius = math.sqrt(max(0.0, 1.0 - z * z))
        angle = index * golden_angle
        direction = Vector((radius * math.cos(angle), radius * math.sin(angle), z))
        hit, location, *_ = _ray_cast(
            cache, position + direction * 1e-4, direction, clearance
        )
        if hit:
            if (location - position).length < clearance - 1e-5:
                return False
    return True


def _make_curve_candidate_groups(cache, components, component_counts, progress=None, cancel=None, cast_rays=True):
    layer_count = int(cache.settings.scientific_layer_count)
    layer_names = _layer_names(layer_count)
    base_layers = _base_layer_indices(layer_count)
    yaw_step = min(math.radians(30.0), cache.horizontal_fov * 0.4)
    yaw_count = max(1, int(math.ceil(math.tau / yaw_step)))
    yaw_step = math.tau / yaw_count
    origin_tolerance = 0.001 * cache.units_per_meter
    used_origins = set()
    groups = []
    stations_by_component = [
        _adaptive_stations(cache, component, component_counts[index])
        for index, component in enumerate(components)
    ]
    intersection_tolerance = max(0.05, float(cache.settings.scientific_minimum_step) * 0.6) * cache.units_per_meter
    for left in range(len(stations_by_component)):
        for right in range(left + 1, len(stations_by_component)):
            for left_station in stations_by_component[left]:
                for right_station in stations_by_component[right]:
                    dx = left_station["point"].x - right_station["point"].x
                    dy = left_station["point"].y - right_station["point"].y
                    dz = abs(left_station["point"].z - right_station["point"].z)
                    if math.hypot(dx, dy) <= intersection_tolerance and dz <= 0.20 * cache.units_per_meter:
                        left_station["critical"] = True
                        right_station["critical"] = True
    unsafe_count = 0
    global_station = 0

    for component_index, component in enumerate(components):
        points = component["points"]
        cumulative = _polyline_metrics(points)
        stations = stations_by_component[component_index]
        source_object = component.get("object")
        manual_layered = bool(source_object and (
            source_object.get("gs_pre_layered_path")
            or source_object.get("gs_manual_valid_segment")
        ))
        manual_layer_count = (
            max(2, int(source_object.get(
                "gs_path_layer_count",
                source_object.get("gs_manual_layer_count", layer_count),
            )))
            if manual_layered else layer_count
        )
        manual_layer_index = (
            max(0, min(manual_layer_count - 1, int(source_object.get(
                "gs_path_layer_index",
                source_object.get("gs_manual_layer_index", 0),
            ))))
            if manual_layered else 0
        )
        manual_layer_name = (
            str(source_object.get(
                "gs_path_layer_name",
                source_object.get("gs_manual_layer_name", f"Layer_{manual_layer_index}"),
            ))
            if manual_layered else ""
        )
        slot_definitions = (
            ((0, 0.0, manual_layer_index),)
            if manual_layered else
            tuple((slot, offset, base_layers[slot]) for slot, offset in enumerate(BASE_OFFSETS))
        )
        for station_index, station in enumerate(stations):
            tangent_yaw = math.atan2(station["tangent"].y, station["tangent"].x)
            phase = tangent_yaw + math.radians(15.0 * global_station)
            for slot, offset, layer_index in slot_definitions:
                distance = _clamp(station["distance"] + offset * station["step"], 0.0, cumulative[-1])
                point, tangent = _point_at(points, cumulative, distance)
                if manual_layered:
                    position = point.copy()
                    candidate_layer_name = manual_layer_name
                else:
                    layers = _layer_z_values(cache, point, layer_count)
                    position = Vector((point.x, point.y, layers[layer_index]))
                    candidate_layer_name = layer_names[layer_index]
                origin_key = _origin_key(position, origin_tolerance)
                if origin_key in used_origins:
                    found = False
                    for attempt in range(1, 17):
                        direction = -1.0 if attempt % 2 else 1.0
                        delta = direction * math.ceil(attempt / 2) * station["step"] * 0.015
                        moved_distance = _clamp(distance + delta, 0.0, cumulative[-1])
                        moved_point, moved_tangent = _point_at(points, cumulative, moved_distance)
                        if manual_layered:
                            moved = moved_point.copy()
                        else:
                            moved_layers = _layer_z_values(cache, moved_point, layer_count)
                            moved = Vector((moved_point.x, moved_point.y, moved_layers[layer_index]))
                        moved_key = _origin_key(moved, origin_tolerance)
                        if moved_key not in used_origins:
                            distance, point, tangent, position, origin_key = moved_distance, moved_point, moved_tangent, moved, moved_key
                            found = True
                            break
                    if not found:
                        continue
                if not _candidate_clear(cache, position):
                    unsafe_count += 1
                    continue
                used_origins.add(origin_key)
                origin_id = f"O{component_index:03d}_{station_index:05d}_{slot}"
                critical = bool(station["critical"])
                base_pitch_degrees = (
                    20.0 if layer_index == 0 else
                    -20.0 if layer_index == manual_layer_count - 1 else
                    0.0
                ) if manual_layered else BASE_PITCHES[slot]
                base = Candidate(
                    candidate_id=f"{origin_id}_BASE",
                    origin_id=origin_id,
                    component_index=component_index,
                    station_index=station_index,
                    distance_along=distance,
                    local_step=station["step"],
                    position=position,
                    tangent=tangent,
                    layer_index=layer_index,
                    layer_name=candidate_layer_name,
                    yaw=(phase + math.radians(BASE_YAWS[slot])) % math.tau,
                    pitch=math.radians(base_pitch_degrees),
                    kind="normal",
                    critical=critical,
                    is_base=True,
                )
                group = [base]
                for yaw_index in range(yaw_count):
                    yaw = yaw_index * yaw_step
                    for pitch_degrees in _allowed_pitches(layer_index, manual_layer_count):
                        candidate = Candidate(
                            candidate_id=f"{origin_id}_Y{yaw_index:02d}_P{pitch_degrees:+.0f}",
                            origin_id=origin_id,
                            component_index=component_index,
                            station_index=station_index,
                            distance_along=distance,
                            local_step=station["step"],
                            position=position.copy(),
                            tangent=tangent.copy(),
                            layer_index=layer_index,
                            layer_name=candidate_layer_name,
                            yaw=yaw,
                            pitch=math.radians(pitch_degrees),
                            kind="normal",
                            critical=critical,
                        )
                        group.append(candidate)
                if critical:
                    for pitch_degrees in (55.0, -55.0):
                        group.append(Candidate(
                            candidate_id=f"{origin_id}_POLAR{pitch_degrees:+.0f}",
                            origin_id=origin_id,
                            component_index=component_index,
                            station_index=station_index,
                            distance_along=distance,
                            local_step=station["step"],
                            position=position.copy(),
                            tangent=tangent.copy(),
                            layer_index=layer_index,
                            layer_name=layer_names[layer_index],
                            yaw=phase % math.tau,
                            pitch=math.radians(pitch_degrees),
                            kind="polar",
                            critical=True,
                        ))
                groups.append(group)
            global_station += 1

    all_candidates = [candidate for group in groups for candidate in group]
    total = max(1, len(all_candidates))
    if cast_rays:
        for index, candidate in enumerate(all_candidates):
            if progress and (index % 16 == 0 or index + 1 == total):
                progress("coverage_rays", index + 1, total)
            _cast_candidate(cache, candidate, progress, cancel)
    return groups, stations_by_component, unsafe_count


class CurveOriginProvider(OriginProvider):
    provider_type = "CURVE"

    def __init__(self, components, component_counts):
        self.components = components
        self.component_counts = component_counts
        self.candidate_groups = []
        self.stations_by_component = []
        self.unsafe_count = 0

    def generate_origins(self):
        groups, stations, unsafe_count = _make_curve_candidate_groups(
            self.cache,
            self.components,
            self.component_counts,
            cast_rays=False,
        )
        self.candidate_groups = groups
        self.stations_by_component = stations
        self.unsafe_count = unsafe_count
        clearance = float(self.settings.scientific_camera_clearance)
        layer_count = int(self.settings.scientific_layer_count)
        seeds = []
        for group in groups:
            representative = next((candidate for candidate in group if candidate.is_base), group[0])
            polar = next((candidate for candidate in group if candidate.kind == "polar"), representative)
            floor_z, ceiling_z = _vertical_limits(self.cache, representative.position)
            component = self.components[representative.component_index]
            source = getattr(component.get("object"), "name", "")
            source_object = component.get("object")
            component_layer_count = (
                max(2, int(source_object.get(
                    "gs_path_layer_count",
                    source_object.get("gs_manual_layer_count", layer_count),
                )))
                if source_object and (
                    source_object.get("gs_pre_layered_path")
                    or source_object.get("gs_manual_valid_segment")
                ) else layer_count
            )
            region_id = f"curve:{representative.component_index}"
            for candidate in group:
                candidate.provider_type = self.provider_type
                candidate.region_id = region_id
            seeds.append(OriginSeed(
                position=representative.position.copy(),
                preferred_direction=_candidate_direction(representative.yaw, representative.pitch),
                local_clearance=clearance,
                floor_z=floor_z,
                ceiling_z=ceiling_z,
                layer_name=representative.layer_name,
                region_id=region_id,
                provider_type=self.provider_type,
                is_critical=representative.critical,
                source_reference=source,
                origin_id=representative.origin_id,
                component_index=representative.component_index,
                station_index=representative.station_index,
                distance_along=representative.distance_along,
                local_step=representative.local_step,
                tangent=representative.tangent.copy(),
                layer_index=representative.layer_index,
                base_yaw=representative.yaw,
                base_pitch=representative.pitch,
                polar_yaw=polar.yaw,
                allowed_pitches=_allowed_pitches(representative.layer_index, component_layer_count),
                allow_polar=any(candidate.kind == "polar" for candidate in group),
            ))
        return seeds


def _groups_from_origin_seeds(cache, seeds):
    yaw_step = min(math.radians(30.0), cache.horizontal_fov * 0.4)
    yaw_count = max(1, int(math.ceil(math.tau / yaw_step)))
    yaw_step = math.tau / yaw_count
    groups = []
    for seed in seeds:
        base = Candidate(
            candidate_id=f"{seed.origin_id}_BASE",
            origin_id=seed.origin_id,
            component_index=seed.component_index,
            station_index=seed.station_index,
            distance_along=seed.distance_along,
            local_step=seed.local_step,
            position=seed.position.copy(),
            tangent=seed.tangent.copy(),
            layer_index=seed.layer_index,
            layer_name=seed.layer_name,
            yaw=seed.base_yaw,
            pitch=seed.base_pitch,
            provider_type=seed.provider_type,
            region_id=seed.region_id,
            critical=seed.is_critical,
            is_base=True,
            distance_band=seed.distance_band,
            view_role=seed.view_role,
        )
        group = [base]
        for yaw_index in range(yaw_count):
            yaw = yaw_index * yaw_step
            for pitch_degrees in seed.allowed_pitches:
                group.append(Candidate(
                    candidate_id=f"{seed.origin_id}_Y{yaw_index:02d}_P{pitch_degrees:+.0f}",
                    origin_id=seed.origin_id,
                    component_index=seed.component_index,
                    station_index=seed.station_index,
                    distance_along=seed.distance_along,
                    local_step=seed.local_step,
                    position=seed.position.copy(),
                    tangent=seed.tangent.copy(),
                    layer_index=seed.layer_index,
                    layer_name=seed.layer_name,
                    yaw=yaw,
                    pitch=math.radians(pitch_degrees),
                    provider_type=seed.provider_type,
                    region_id=seed.region_id,
                    critical=seed.is_critical,
                    distance_band=seed.distance_band,
                    view_role=seed.view_role,
                ))
        if seed.allow_polar:
            for pitch_degrees in (55.0, -55.0):
                group.append(Candidate(
                    candidate_id=f"{seed.origin_id}_POLAR{pitch_degrees:+.0f}",
                    origin_id=seed.origin_id,
                    component_index=seed.component_index,
                    station_index=seed.station_index,
                    distance_along=seed.distance_along,
                    local_step=seed.local_step,
                    position=seed.position.copy(),
                    tangent=seed.tangent.copy(),
                    layer_index=seed.layer_index,
                    layer_name=seed.layer_name,
                    yaw=seed.polar_yaw,
                    pitch=math.radians(pitch_degrees),
                    provider_type=seed.provider_type,
                    region_id=seed.region_id,
                    kind="polar",
                    critical=True,
                    distance_band=seed.distance_band,
                    view_role=seed.view_role,
                ))
        groups.append(group)
    return groups


class FreeSpaceOriginProvider(OriginProvider):
    provider_type = "FREE_SPACE"

    def __init__(self, adaptive_small_spaces=False):
        self.adaptive_small_spaces = adaptive_small_spaces
        self.candidate_groups = []
        self.stations_by_component = []
        self.unsafe_count = 0
        self.too_close_count = 0
        self.metadata = {}

    def generate_origins(self):
        free_map = free_space_planner.build_free_space_map(
            self.cache,
            self.settings,
            _ray_cast,
            cancel=getattr(self, "cancel", None),
            progress=getattr(self, "progress", None),
        )
        self.cache.free_space_map = free_map
        planar_origins = free_space_planner.generate_candidate_origins(
            free_map, self.settings, self.cache.units_per_meter
        )
        seeds = []
        region_stations = [[] for _region in free_map.regions]
        layer_total = 0
        for planar_index, item in enumerate(planar_origins):
            cell = item.cell
            region = free_map.regions[cell.component_id]
            clearance_m = cell.clearance / max(1e-9, self.cache.units_per_meter)
            unsuitable_distance = float(
                getattr(self.settings, "near_field_unsuitable_distance", 0.35)
            )
            protect_near_field = bool(getattr(self.settings, "near_field_protection", True))
            if (
                protect_near_field
                and item.distance_band == "NEAR"
                and item.view_role != "door_detail"
                and clearance_m < unsuitable_distance
            ):
                rejected_id = f"FS_REJECT_R{cell.component_id:03d}_O{planar_index:05d}"
                self.cache.rejected_too_close_candidate_ids.add(rejected_id)
                self.cache.near_field_rejected_positions[rejected_id] = Vector((cell.x, cell.y, (cell.floor_z + cell.ceiling_z) * 0.5))
                self.too_close_count += 1
                continue
            use_two_layers = self.adaptive_small_spaces and region.classification in {
                "SMALL_ROOM", "NARROW_CORRIDOR", "DOORWAY_TRANSITION",
            }
            layer_count = 2 if use_two_layers else int(self.settings.scientific_layer_count)
            layer_names = _layer_names(layer_count)
            point = Vector((cell.x, cell.y, (cell.floor_z + cell.ceiling_z) * 0.5))
            layer_values = _layer_z_values(self.cache, point, layer_count)
            preferred = item.preferred_direction.copy()
            if region.classification == "NARROW_CORRIDOR" and item.source != "doorway_tier":
                preferred = (region.principal_axis + region.cross_axis * (0.45 if planar_index % 2 else -0.45)).normalized()
            preferred_yaw = math.atan2(preferred.y, preferred.x) % math.tau
            dynamic_clearance = float(self.settings.scientific_camera_clearance)
            if use_two_layers:
                dynamic_clearance = min(
                    dynamic_clearance,
                    max(0.08, region.minimum_width_m * 0.18),
                )
            if use_two_layers or region.near_field:
                overlap = float(
                    getattr(self.settings, "near_field_doorway_target_overlap", 0.85)
                    if item.source == "doorway_tier" or region.near_field
                    else 0.825
                )
                base_step_m = _clamp(
                    region.minimum_width_m * (1.0 - overlap), 0.15, 0.25
                )
            else:
                base_step_m = max(
                    free_map.resolution / self.cache.units_per_meter,
                    float(getattr(self.settings, "free_space_candidate_spacing", 0.75)),
                )
            step_ratio = _clamp(
                float(getattr(self.settings, "near_field_step_distance_ratio", 0.25)),
                0.20, 0.30,
            )
            minimum_spacing_m = float(
                getattr(self.settings, "near_field_minimum_origin_spacing", 0.12)
            )
            if region.near_field or item.distance_band in {"NEAR", "MID"}:
                base_step_m = max(
                    minimum_spacing_m,
                    min(base_step_m, clearance_m * step_ratio),
                )
            local_step = base_step_m * self.cache.units_per_meter
            station = {
                "distance": planar_index * local_step,
                "point": point,
                "tangent": preferred,
                "step": local_step,
                "depth": cell.clearance,
                "critical": item.critical,
            }
            region_stations[cell.component_id].append(station)
            for layer_index, z_value in enumerate(layer_values):
                position = Vector((cell.x, cell.y, z_value))
                if not _candidate_clear(self.cache, position, clearance_m=dynamic_clearance):
                    self.unsafe_count += 1
                    continue
                if layer_count == 2:
                    base_pitch = math.radians(15.0 if layer_index == 0 else -15.0)
                else:
                    base_pitch = math.radians(20.0 if layer_index == 0 else -20.0 if layer_index == layer_count - 1 else 0.0)
                origin_id = f"FS_R{cell.component_id:03d}_O{planar_index:05d}_L{layer_index}"
                seeds.append(OriginSeed(
                    position=position,
                    preferred_direction=preferred,
                    local_clearance=cell.clearance / self.cache.units_per_meter,
                    floor_z=cell.floor_z,
                    ceiling_z=cell.ceiling_z,
                    layer_name=layer_names[layer_index],
                    region_id=region.region_id,
                    provider_type=self.provider_type,
                    is_critical=item.critical,
                    source_reference=item.source,
                    origin_id=origin_id,
                    component_index=cell.component_id,
                    station_index=planar_index,
                    distance_along=planar_index * local_step,
                    local_step=local_step,
                    tangent=preferred,
                    layer_index=layer_index,
                    base_yaw=preferred_yaw,
                    base_pitch=base_pitch,
                    polar_yaw=preferred_yaw,
                    allowed_pitches=_allowed_pitches(layer_index, layer_count),
                    allow_polar=item.critical,
                    distance_band=item.distance_band,
                    view_role=item.view_role,
                ))
                layer_total += 1
        self.stations_by_component = region_stations
        self.candidate_groups = _groups_from_origin_seeds(self.cache, seeds)
        counts = {}
        band_counts = {"NEAR": 0, "MID": 0, "FAR": 0, "REGULAR": 0}
        for seed in seeds:
            band_counts[seed.distance_band] = band_counts.get(seed.distance_band, 0) + 1
        for region in free_map.regions:
            counts[region.classification] = counts.get(region.classification, 0) + 1
            if region.near_field:
                counts["NEAR_FIELD"] = counts.get("NEAR_FIELD", 0) + 1
        self.metadata = {
            "mode": self.provider_type,
            "providers": [self.provider_type],
            "free_space_component_count": len(free_map.regions),
            "free_space_area_m2": free_map.area_m2,
            "generated_origin_count": layer_total + self.unsafe_count + self.too_close_count,
            "valid_origin_count": len(seeds),
            "provider_origin_counts": {self.provider_type: len(seeds)},
            "free_space_grid_resolution_m": free_map.resolution_m,
            "free_space_grid_cell_count": len(free_map.cells),
            "free_space_invalid_cell_count": free_map.invalid_cell_count,
            "doorway_node_count": len(free_map.doorway_nodes),
            "medial_axis_node_count": len(free_map.medial_axis_nodes),
            "space_class_counts": counts,
            "near_mid_far_origin_counts": band_counts,
            "rejected_too_close_origin_count": self.too_close_count,
            "near_field_region_count": counts.get("NEAR_FIELD", 0),
        }
        return seeds


class HybridOriginProvider(OriginProvider):
    provider_type = "HYBRID"

    def __init__(self, components, component_counts):
        self.components = components
        self.component_counts = component_counts
        self.candidate_groups = []
        self.stations_by_component = []
        self.unsafe_count = 0
        self.metadata = {}

    def generate_origins(self):
        curve = CurveOriginProvider(self.components, self.component_counts)
        free = FreeSpaceOriginProvider(adaptive_small_spaces=True)
        for provider in (curve, free):
            provider.prepare(self.context, self.cache, self.settings)
            provider.progress = getattr(self, "progress", None)
            provider.cancel = getattr(self, "cancel", None)
        curve_seeds = curve.generate_origins() if self.components else []
        free_seeds = free.generate_origins()
        duplicate_distance = 0.05 * self.cache.units_per_meter
        curve_positions = [seed.position for seed in curve_seeds]
        kept_free = [
            seed for seed in free_seeds
            if not any((seed.position - position).length < duplicate_distance for position in curve_positions)
        ]
        kept_ids = {seed.origin_id for seed in kept_free}
        self.candidate_groups = list(curve.candidate_groups)
        self.candidate_groups.extend(
            group for group in free.candidate_groups
            if group and group[0].origin_id in kept_ids
        )
        self.stations_by_component = list(curve.stations_by_component) + list(free.stations_by_component)
        self.unsafe_count = curve.unsafe_count + free.unsafe_count
        free_metadata = dict(free.metadata)
        free_metadata.update({
            "mode": self.provider_type,
            "providers": ["CURVE", "FREE_SPACE"] if self.components else ["FREE_SPACE"],
            "generated_origin_count": len(curve_seeds) + free_metadata.get("generated_origin_count", len(free_seeds)),
            "valid_origin_count": len(curve_seeds) + len(kept_free),
            "provider_origin_counts": {
                "CURVE": len(curve_seeds),
                "FREE_SPACE": len(kept_free),
            },
            "hybrid_deduplicated_origin_count": len(free_seeds) - len(kept_free),
        })
        self.metadata = free_metadata
        return curve_seeds + kept_free


class CoverageDrivenOriginProvider(OriginProvider):
    provider_type = "COVERAGE_DRIVEN"

    def __init__(self, reachable, existing_coverage, selected):
        self.reachable = reachable
        self.existing_coverage = existing_coverage
        self.selected = selected
        self.candidate_groups = []
        self.stations_by_component = []
        self.unsafe_count = 0
        self.target_count = 0

    def _target_observations(self):
        minimum = int(self.settings.scientific_minimum_observations)
        maximum = max(1, int(getattr(self.settings, "coverage_driven_max_surface_targets", 48)))
        ranked = []
        for key, observation in self.reachable.items():
            count = self.existing_coverage.get(key, {}).get("observation_count", 0)
            if count >= minimum:
                continue
            ranked.append((count, observation.object_name, observation.face_index, repr(key), observation))
        ranked.sort(key=lambda item: item[:-1])
        spacing = max(self.cache.coverage_cell_size * 4.0, 0.10 * self.cache.units_per_meter)
        selected = []
        for _count, _name, _face, _key, observation in ranked:
            if any((observation.position - old.position).length < spacing for old in selected):
                continue
            selected.append(observation)
            if len(selected) >= maximum:
                break
        return selected

    def generate_origins(self):
        if self.cache.free_space_map is None:
            free_map = free_space_planner.build_free_space_map(
                self.cache,
                self.settings,
                _ray_cast,
                cancel=getattr(self, "cancel", None),
                progress=getattr(self, "progress", None),
            )
            self.cache.free_space_map = free_map
        free_map = self.cache.free_space_map
        targets = self._target_observations()
        self.target_count = len(targets)
        maximum = max(0, int(getattr(self.settings, "coverage_driven_max_cameras", 12)))
        per_surface = max(1, int(getattr(self.settings, "coverage_driven_candidates_per_surface", 3)))
        clearance_m = float(self.settings.scientific_camera_clearance)
        duplicate_distance = 0.08 * self.cache.units_per_meter
        existing_positions = [candidate.position for candidate in self.selected]
        seeds = []
        for target_index, observation in enumerate(targets):
            if len(seeds) >= maximum:
                break
            view_side = observation.view_direction.normalized()
            tangent = view_side.cross(Vector((0.0, 0.0, 1.0)))
            if tangent.length <= 1e-8:
                tangent = Vector((1.0, 0.0, 0.0))
            tangent.normalize()
            proposals = []
            for distance_m in (0.70, 1.10, 1.60):
                distance = distance_m * self.cache.units_per_meter
                lateral = 0.18 * distance
                proposals.extend((
                    observation.position + view_side * distance,
                    observation.position + view_side * distance + tangent * lateral,
                    observation.position + view_side * distance - tangent * lateral,
                ))
            made_for_surface = 0
            for proposal in proposals:
                if len(seeds) >= maximum or made_for_surface >= per_surface:
                    break
                for cell in free_space_planner.nearest_cells(free_map, proposal, maximum=8):
                    low = cell.floor_z + clearance_m * self.cache.units_per_meter
                    high = cell.ceiling_z - clearance_m * self.cache.units_per_meter
                    if high <= low:
                        continue
                    position = Vector((cell.x, cell.y, _clamp(proposal.z, low, high)))
                    if any((position - old).length < duplicate_distance for old in existing_positions):
                        continue
                    if any((position - seed.position).length < duplicate_distance for seed in seeds):
                        continue
                    if not _candidate_clear(self.cache, position, clearance_m=clearance_m):
                        self.unsafe_count += 1
                        continue
                    direction = observation.position - position
                    if direction.length <= 1e-8:
                        continue
                    direction.normalize()
                    yaw = math.atan2(direction.y, direction.x) % math.tau
                    pitch = math.asin(_clamp(direction.z, -1.0, 1.0))
                    pitch_degrees = math.degrees(pitch)
                    allowed = tuple(sorted({
                        _clamp(pitch_degrees + offset, -55.0, 55.0)
                        for offset in (-12.0, 0.0, 12.0)
                    }))
                    origin_id = f"CD_T{target_index:04d}_O{made_for_surface:02d}"
                    seeds.append(OriginSeed(
                        position=position,
                        preferred_direction=direction,
                        local_clearance=cell.clearance / self.cache.units_per_meter,
                        floor_z=cell.floor_z,
                        ceiling_z=cell.ceiling_z,
                        layer_name="Coverage",
                        region_id=f"coverage:{observation.object_name}",
                        provider_type=self.provider_type,
                        is_critical=True,
                        source_reference=repr(observation.key),
                        origin_id=origin_id,
                        component_index=cell.component_id,
                        station_index=target_index,
                        distance_along=0.0,
                        local_step=0.25 * self.cache.units_per_meter,
                        tangent=direction,
                        layer_index=0,
                        base_yaw=yaw,
                        base_pitch=pitch,
                        polar_yaw=yaw,
                        allowed_pitches=allowed,
                        allow_polar=observation.category in {"floor_like", "ceiling_like"},
                    ))
                    existing_positions.append(position)
                    made_for_surface += 1
                    break
        self.candidate_groups = _groups_from_origin_seeds(self.cache, seeds)
        self.stations_by_component = [[{
            "distance": 0.0,
            "point": seed.position,
            "tangent": seed.tangent,
            "step": seed.local_step,
            "depth": seed.local_clearance * self.cache.units_per_meter,
            "critical": True,
        } for seed in seeds]]
        return seeds


def _make_candidate_groups(
    cache, components, component_counts, progress=None, cancel=None, cast_rays=True,
    origin_provider=None,
):
    provider = origin_provider or CurveOriginProvider(components, component_counts)
    provider.prepare(cache.scene, cache, cache.settings)
    provider.progress = progress
    provider.cancel = cancel
    seeds = provider.generate_origins()
    groups = provider.candidate_groups
    all_candidates = [candidate for group in groups for candidate in group]
    total = max(1, len(all_candidates))
    if cast_rays:
        for index, candidate in enumerate(all_candidates):
            if progress and (index % 16 == 0 or index + 1 == total):
                progress("coverage_rays", index + 1, total)
            _cast_candidate(cache, candidate, progress, cancel)
    cache.origin_generation = dict(getattr(provider, "metadata", {}) or {
            "mode": provider.provider_type,
            "providers": [provider.provider_type],
            "free_space_component_count": 0,
            "free_space_area_m2": 0.0,
            "generated_origin_count": len(seeds) + provider.unsafe_count,
            "valid_origin_count": len(seeds),
            "provider_origin_counts": {provider.provider_type: len(seeds)},
        }
    )
    return groups, provider.stations_by_component, provider.unsafe_count



def _angle_delta(a, b):
    return abs((a - b + math.pi) % math.tau - math.pi)


NEAR_FIELD_REJECT_SCORE = -1.0e9


def _near_field_mid_neighbors(cache, candidate, selected):
    candidate_cells = _overlap_cell_keys(candidate)
    if not candidate_cells:
        return 0, 0.0
    required_overlap = float(
        getattr(cache.settings, "near_field_minimum_mid_overlap", 0.35)
    )
    minimum_baseline = max(
        float(getattr(cache.settings, "near_field_minimum_origin_spacing", 0.12)),
        float(getattr(cache.settings, "near_field_minimum_baseline", 0.20)),
    ) * cache.units_per_meter
    count = 0
    ratios = []
    for other in selected or ():
        if other.distance_band == "NEAR":
            continue
        other_cells = _overlap_cell_keys(other)
        if not other_cells:
            continue
        common = len(candidate_cells & other_cells)
        ratio = common / max(1, min(len(candidate_cells), len(other_cells)))
        baseline = (candidate.position - other.position).length
        if ratio >= required_overlap and baseline >= minimum_baseline:
            count += 1
            ratios.append(ratio)
    return count, (sum(ratios) / len(ratios) if ratios else 0.0)


def _score_candidate(
    cache, candidate, coverage, previous, minimum_obs, preferred_obs, selected=None,
):
    max_incidence = float(cache.settings.scientific_maximum_incidence_angle)
    observations = [obs for obs in candidate.observations.values() if obs.incidence_angle <= max_incidence]
    total = max(1, len(observations))
    new_gain = 0
    under_gain = 0.0
    strict_under_gain = 0.0
    diversity_gain = 0
    for observation in observations:
        current = coverage.get(observation.key)
        if current is None:
            new_gain += 1
            under_gain += 1.0
            strict_under_gain += 1.0
            diversity_gain += 1
            continue
        count = current["observation_count"]
        if count < minimum_obs:
            deficit = (minimum_obs - count) / max(1, minimum_obs)
            under_gain += deficit
            strict_under_gain += deficit
        elif count < preferred_obs:
            under_gain += 0.2 * (preferred_obs - count) / max(1, preferred_obs - minimum_obs)
        if observation.direction_bin not in current["view_direction_bins"]:
            diversity_gain += 1
    coverage_gain = new_gain / total
    under_gain /= total
    strict_under_gain /= total
    diversity_gain /= total
    parallax = 0.0
    continuity = 0.5
    redundancy_penalty = 0.0
    if previous is not None:
        baseline = (candidate.position - previous.position).length
        useful = float(cache.settings.scientific_camera_clearance) * cache.units_per_meter
        parallax = _clamp(baseline / max(1e-6, useful * 2.0), 0.0, 1.0)
        continuity = 1.0 - _clamp(_angle_delta(candidate.yaw, previous.yaw) / math.pi, 0.0, 1.0)
        if baseline < 0.01 * cache.units_per_meter and _angle_delta(candidate.yaw, previous.yaw) < math.radians(5.0):
            redundancy_penalty = 0.75
    distances = [obs.distance for obs in observations]
    extreme_penalty = 0.0
    if distances:
        median = _percentile(distances, 0.5)
        near = float(cache.settings.scientific_camera_clearance) * cache.units_per_meter
        if median < near * 1.25 or median > cache.scene_diagonal:
            extreme_penalty = 0.15
    else:
        extreme_penalty = 0.8
    near_field_adjustment = 0.0
    protect_near_field = bool(getattr(cache.settings, "near_field_protection", True)) and candidate.provider_type != "CURVE"
    if protect_near_field:
        cache.rejected_too_close_candidate_ids.discard(candidate.candidate_id)
        cache.dominant_surface_rejected_ids.discard(candidate.candidate_id)
        cache.near_field_quota_rejected_ids.discard(candidate.candidate_id)
        cache.near_field_rejected_positions.pop(candidate.candidate_id, None)
        recommended_min = float(
            getattr(cache.settings, "near_field_recommended_distance_min", 0.60)
        )
        recommended_max = max(
            recommended_min,
            float(getattr(cache.settings, "near_field_recommended_distance_max", 1.00)),
        )
        dominant_threshold = float(
            getattr(cache.settings, "near_field_dominant_surface_ratio", 0.65)
        )
        minimum_environment = float(
            getattr(cache.settings, "near_field_minimum_environment_ratio", 0.35)
        )
        required_neighbors = int(
            getattr(cache.settings, "near_field_required_mid_neighbors", 2)
        )
        strong_mid_count, mid_overlap = _near_field_mid_neighbors(
            cache, candidate, selected
        )
        candidate.strong_mid_overlap_count = strong_mid_count
        candidate.near_field_average_overlap = mid_overlap
        candidate.shared_environment_ratio = mid_overlap
        severe_dominance = (
            candidate.dominant_near_surface_ratio >= dominant_threshold
            and (
                mid_overlap < minimum_environment
                or strong_mid_count < required_neighbors
            )
        )
        unsuitable_distance = float(
            getattr(cache.settings, "near_field_unsuitable_distance", 0.35)
        )
        too_close = (
            0.0 < candidate.reference_target_distance < unsuitable_distance
            and candidate.view_role != "door_detail"
        )
        minimum_gain = float(
            getattr(cache.settings, "near_field_minimum_under_observed_gain", 0.05)
        )
        under_covered_benefit = (
            coverage_gain >= minimum_gain or strict_under_gain >= 0.10
        )
        protected_exception = (
            under_covered_benefit
            and strong_mid_count >= required_neighbors
            and mid_overlap >= minimum_environment
        )
        if (severe_dominance or too_close) and not protected_exception:
            candidate.near_field_rejected = True
            candidate.near_field_rejection_reason = (
                "dominant_near_surface" if severe_dominance else "recommended_distance"
            )
            cache.near_field_rejected_positions[candidate.candidate_id] = candidate.position.copy()
            if severe_dominance:
                cache.dominant_surface_rejected_ids.add(candidate.candidate_id)
            if too_close:
                cache.rejected_too_close_candidate_ids.add(candidate.candidate_id)
            return NEAR_FIELD_REJECT_SCORE
        candidate.near_field_rejected = False
        candidate.near_field_rejection_reason = ""
        if candidate.reference_target_distance < recommended_min:
            near_field_adjustment -= 0.25 * (
                1.0 - candidate.reference_target_distance / max(1e-6, recommended_min)
            )
        elif candidate.reference_target_distance <= recommended_max:
            near_field_adjustment += 0.04
        if candidate.distance_band == "NEAR":
            near_field_adjustment -= 0.25 * candidate.dominant_near_surface_ratio
            near_field_adjustment += 0.08 * _clamp(
                mid_overlap / max(1e-6, float(getattr(
                    cache.settings, "near_field_minimum_mid_overlap", 0.35
                ))), 0.0, 1.0,
            )
        if previous is not None and candidate.reference_target_distance > 0.0 and previous.reference_target_distance > 0.0:
            scale_ratio = min(candidate.reference_target_distance, previous.reference_target_distance) / max(candidate.reference_target_distance, previous.reference_target_distance)
            near_field_adjustment += 0.05 * scale_ratio
    base_bonus = 0.01 if candidate.is_base else 0.0
    polar_bonus = 0.03 if candidate.kind == "polar" and candidate.critical else 0.0
    provider_bonus = 0.08 if candidate.provider_type == "COVERAGE_DRIVEN" else 0.0
    return (
        0.45 * coverage_gain
        + 0.25 * under_gain
        + 0.15 * diversity_gain
        + 0.10 * parallax
        + 0.05 * continuity
        + base_bonus
        + polar_bonus
        + provider_bonus
        + near_field_adjustment
        - redundancy_penalty
        - extreme_penalty
    )


def _quality_cell_map(candidate, maximum_incidence):
    cached = candidate.quality_maps.get(maximum_incidence)
    if cached is not None:
        return cached
    quality_hits = {
        key: observation for key, observation in candidate.observations.items()
        if observation.incidence_angle <= maximum_incidence
    }
    cached = _expanded_surface_map(quality_hits)
    candidate.quality_maps[maximum_incidence] = cached
    return cached


def _apply_candidate(coverage, candidate, maximum_incidence):
    for key, observation in _quality_cell_map(candidate, maximum_incidence).items():
        current = coverage.get(key)
        if current is None:
            current = {
                "observation_count": 0,
                "best_incidence_angle": 180.0,
                "observed_distance_min": float("inf"),
                "observed_distance_max": 0.0,
                "view_direction_bins": set(),
                "candidate_camera_ids": [],
                "category": observation.category,
                "position": observation.position.copy(),
                "object_name": observation.object_name,
            }
            coverage[key] = current
        current["observation_count"] += 1
        current["best_incidence_angle"] = min(current["best_incidence_angle"], observation.incidence_angle)
        current["observed_distance_min"] = min(current["observed_distance_min"], observation.distance)
        current["observed_distance_max"] = max(current["observed_distance_max"], observation.distance)
        current["view_direction_bins"].add(observation.direction_bin)
        current["candidate_camera_ids"].append(candidate.candidate_id)


def _even_indices(total, count):
    if count >= total:
        return list(range(total))
    if count <= 1:
        return [0]
    raw = [int(round(index * (total - 1) / (count - 1))) for index in range(count)]
    result = []
    used = set()
    for value in raw:
        while value in used and value + 1 < total:
            value += 1
        if value not in used:
            used.add(value)
            result.append(value)
    return result


def _distributed_group_indices(groups, count):
    """Select complete six-slot stations before using individual leftovers."""
    if count >= len(groups):
        return list(range(len(groups)))
    stations = []
    station_lookup = {}
    for group_index, group in enumerate(groups):
        if not group:
            continue
        key = (group[0].provider_type, group[0].component_index, group[0].station_index)
        bucket = station_lookup.get(key)
        if bucket is None:
            bucket = []
            station_lookup[key] = bucket
            stations.append(bucket)
        bucket.append(group_index)
    complete_count = min(len(stations), count // 6)
    selected = []
    for station_index in _even_indices(len(stations), complete_count) if complete_count else ():
        selected.extend(stations[station_index])
    selected = selected[:count]
    if len(selected) < count:
        selected_set = set(selected)
        remaining = [index for index in range(len(groups)) if index not in selected_set]
        needed = min(count - len(selected), len(remaining))
        selected.extend(remaining[index] for index in _even_indices(len(remaining), needed))
    return sorted(selected)


def _group_distance_priority(group):
    if not group:
        return 3
    if all(candidate.provider_type == "CURVE" for candidate in group):
        return 1
    bands = {candidate.distance_band for candidate in group}
    if "FAR" in bands:
        return 0
    if "MID" in bands or "REGULAR" in bands:
        return 1
    return 2


def _near_field_initial_group_indices(groups, count):
    if count >= len(groups):
        return sorted(range(len(groups)), key=lambda index: (
            _group_distance_priority(groups[index]), index,
        ))
    non_near = [
        index for index, group in enumerate(groups)
        if _group_distance_priority(group) < 2
    ]
    near = [
        index for index, group in enumerate(groups)
        if _group_distance_priority(group) == 2
    ]
    selected = []
    non_near_count = min(count, len(non_near))
    if non_near_count:
        selected.extend(
            non_near[index] for index in _even_indices(len(non_near), non_near_count)
        )
    remaining = count - len(selected)
    if remaining and near:
        selected.extend(near[index] for index in _even_indices(len(near), min(remaining, len(near))))
    return sorted(selected, key=lambda index: (
        _group_distance_priority(groups[index]), index,
    ))


def _best_from_group(
    cache, group, coverage, previous, minimum_obs, preferred_obs,
    force_kind=None, selected=None,
):
    candidates = [item for item in group if force_kind is None or item.kind == force_kind]
    if not candidates:
        return None
    auto_coverage = bool(getattr(cache.settings, "scientific_auto_coverage", True))
    protect_near_field = bool(getattr(cache.settings, "near_field_protection", True))
    if not auto_coverage and (
        not protect_near_field or all(item.provider_type == "CURVE" for item in candidates)
    ):
        return next((item for item in candidates if item.is_base), candidates[0])
    ranked = []
    ordered = sorted(candidates, key=lambda item: (not item.is_base, item.candidate_id))
    for candidate in ordered:
        candidate.score = _score_candidate(
            cache, candidate, coverage, previous, minimum_obs, preferred_obs,
            selected=selected,
        )
        if candidate.score <= NEAR_FIELD_REJECT_SCORE:
            continue
        if not auto_coverage:
            return candidate
        ranked.append(candidate)
    if not ranked:
        return None
    return max(ranked, key=lambda item: (item.score, item.is_base, -item.pitch, -item.yaw, item.candidate_id))


def _all_visible_cells(groups):
    result = {}
    for group in groups:
        for candidate in group:
            for key, observation in candidate.observations.items():
                old = result.get(key)
                if old is None or observation.incidence_angle < old.incidence_angle:
                    result[key] = observation
    return result


def _category_ratio(coverage, visible, category):
    total = sum(1 for observation in visible.values() if observation.category == category)
    if total == 0:
        return 1.0
    observed = sum(1 for key, observation in visible.items() if observation.category == category and key in coverage)
    return observed / total


def _overlap_cell_keys(candidate, block=8):
    cached = candidate.overlap_cells.get(block)
    if cached is not None:
        return cached
    keys = set()
    for key in candidate.observations:
        object_name, face_index, position_bin, normal_bin = key
        coarse_position = tuple(value // block for value in position_bin)
        keys.add((object_name, face_index, coarse_position, normal_bin))
    candidate.overlap_cells[block] = keys
    return keys


def _overlap_graph(candidates, recommended=0.50, cancel=None):
    cells = [_overlap_cell_keys(candidate) for candidate in candidates]
    graph = {index: set() for index in range(len(candidates))}
    strong_neighbors = [0] * len(candidates)
    edges = []
    spatial_rights = None
    if len(candidates) > 1000:
        steps = [candidate.local_step for candidate in candidates if candidate.local_step > 1e-9]
        bucket_size = max(1e-6, (_percentile(steps, 0.5) or 1.0) * 6.0)
        buckets = {}
        candidate_buckets = []
        for index, candidate in enumerate(candidates):
            key = tuple(int(math.floor(value / bucket_size)) for value in candidate.position)
            candidate_buckets.append(key)
            buckets.setdefault(key, []).append(index)
        spatial_rights = []
        for left, key in enumerate(candidate_buckets):
            nearby = set()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        nearby.update(buckets.get((key[0] + dx, key[1] + dy, key[2] + dz), ()))
            spatial_rights.append(sorted(index for index in nearby if index > left))
    for left in range(len(candidates)):
        if left % 32 == 0 and cancel and cancel():
            raise PlanningCancelled("Scientific camera planning cancelled")
        if not cells[left]:
            continue
        rights = (
            spatial_rights[left] if spatial_rights is not None
            else range(left + 1, len(candidates))
        )
        for right in rights:
            if not cells[right]:
                continue
            common = len(cells[left] & cells[right])
            if not common:
                continue
            ratio = common / max(1, min(len(cells[left]), len(cells[right])))
            if ratio >= 0.10:
                graph[left].add(right)
                graph[right].add(left)
                edges.append((left, right, ratio))
            baseline = (candidates[left].position - candidates[right].position).length
            useful_baseline = 0.10 * min(
                candidates[left].local_step, candidates[right].local_step
            )
            if ratio >= recommended and baseline >= useful_baseline:
                strong_neighbors[left] += 1
                strong_neighbors[right] += 1
    components = []
    remaining = set(graph)
    while remaining:
        seed = min(remaining)
        stack = [seed]
        remaining.remove(seed)
        component = []
        while stack:
            item = stack.pop()
            component.append(item)
            for neighbor in graph[item]:
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
        components.append(sorted(component))
    return graph, components, edges, strong_neighbors


def _near_field_camera_limit(settings, budget):
    ratio = _clamp(float(getattr(
        settings, "near_field_maximum_camera_ratio", 0.15
    )), 0.0, 0.50)
    if ratio <= 0.0:
        return 0
    return max(1, int(math.floor(max(0, budget) * ratio)))


def _select_candidates(cache, groups, legacy_budget, progress=None, cancel=None):
    multiplier = float(cache.settings.scientific_view_budget_multiplier)
    target_budget = max(1, int(round(legacy_budget * multiplier)))
    target_budget = min(target_budget, len(groups))
    reserve = min(target_budget - 1, max(0, int(cache.coverage_driven_reserve)))
    initial_budget = max(1, target_budget - reserve)
    protect_near_field = bool(getattr(cache.settings, "near_field_protection", True)) and any(
        candidate.provider_type != "CURVE" for group in groups for candidate in group
    )
    selected_group_indices = (
        _near_field_initial_group_indices(groups, initial_budget)
        if protect_near_field else _distributed_group_indices(groups, initial_budget)
    )
    required_layers = sorted({group[0].layer_name for group in groups if group})
    layer_counts = {}
    for index in selected_group_indices:
        layer = groups[index][0].layer_name
        layer_counts[layer] = layer_counts.get(layer, 0) + 1
    for missing_layer in required_layers:
        if layer_counts.get(missing_layer, 0) > 0:
            continue
        replacement = next((
            position for position in range(len(selected_group_indices) - 1, -1, -1)
            if layer_counts.get(groups[selected_group_indices[position]][0].layer_name, 0) > 1
        ), None)
        candidate_index = next((
            index for index in sorted(
                range(len(groups)),
                key=lambda item: (_group_distance_priority(groups[item]), item),
            )
            for group in (groups[index],)
            if group and group[0].layer_name == missing_layer and index not in selected_group_indices
        ), None)
        if replacement is not None and candidate_index is not None:
            old_layer = groups[selected_group_indices[replacement]][0].layer_name
            layer_counts[old_layer] -= 1
            selected_group_indices[replacement] = candidate_index
            layer_counts[missing_layer] = 1
    selected_group_indices.sort(key=lambda index: (
        _group_distance_priority(groups[index]), index,
    ))
    selected_groups = set()
    selected = []
    coverage = {}
    minimum_obs = int(cache.settings.scientific_minimum_observations)
    preferred_obs = max(minimum_obs, int(cache.settings.scientific_preferred_observations))
    maximum_incidence = float(cache.settings.scientific_maximum_incidence_angle)
    previous = None
    maximum_near_count = _near_field_camera_limit(cache.settings, target_budget)

    def add_group(index, force_kind=None, forced_candidate=None):
        nonlocal previous
        if index in selected_groups:
            return False
        candidate = forced_candidate
        if candidate is not None and candidate.provider_type != "CURVE" and protect_near_field:
            candidate.score = _score_candidate(
                cache, candidate, coverage, previous, minimum_obs, preferred_obs,
                selected=selected,
            )
            if candidate.score <= NEAR_FIELD_REJECT_SCORE:
                return False
        if candidate is None:
            candidate = _best_from_group(
                cache, groups[index], coverage, previous, minimum_obs,
                preferred_obs, force_kind, selected=selected,
            )
        if candidate is None:
            return False
        if protect_near_field and candidate.distance_band == "NEAR":
            near_count = sum(item.distance_band == "NEAR" for item in selected)
            if near_count >= maximum_near_count:
                candidate.near_field_rejected = True
                candidate.near_field_rejection_reason = "near_camera_quota"
                cache.near_field_quota_rejected_ids.add(candidate.candidate_id)
                cache.near_field_rejected_positions[candidate.candidate_id] = candidate.position.copy()
                return False
        selected_groups.add(index)
        selected.append(candidate)
        _apply_candidate(coverage, candidate, maximum_incidence)
        previous = candidate
        return True

    for ordinal, index in enumerate(selected_group_indices):
        if cancel and cancel():
            raise PlanningCancelled("Scientific camera planning cancelled")
        if progress and (ordinal % 8 == 0 or ordinal + 1 == len(selected_group_indices)):
            progress("greedy_selection", ordinal + 1, len(selected_group_indices))
        base_candidate = next((
            candidate for candidate in groups[index]
            if candidate.is_base and candidate.provider_type == "CURVE"
        ), None)
        add_group(index, forced_candidate=base_candidate)

    polar_added = 0
    if getattr(cache.settings, "scientific_auto_floor_ceiling", True) and len(selected) < target_budget:
        visible = _all_visible_cells(groups)
        missing = []
        if _category_ratio(coverage, visible, "floor_like") < 0.95:
            missing.append(-55.0)
        if _category_ratio(coverage, visible, "ceiling_like") < 0.95:
            missing.append(55.0)
        for wanted_pitch in missing:
            ranked = []
            category = "ceiling_like" if wanted_pitch > 0 else "floor_like"
            for index, group in enumerate(groups):
                if index in selected_groups:
                    continue
                for candidate in group:
                    if candidate.kind != "polar" or math.copysign(1.0, candidate.pitch) != math.copysign(1.0, wanted_pitch):
                        continue
                    gain = sum(1 for obs in candidate.observations.values() if obs.category == category and obs.key not in coverage)
                    ranked.append((gain, -index, candidate.candidate_id, index, candidate))
            for _gain, _neg_index, _candidate_id, index, candidate in sorted(ranked, reverse=True):
                if len(selected) >= target_budget:
                    break
                if add_group(index, "polar", forced_candidate=candidate):
                    polar_added += 1
                    break

    bridge_added = 0
    recommended_overlap = float(cache.settings.scientific_minimum_overlap)
    origin_group_indices = {
        group[0].origin_id: index for index, group in enumerate(groups) if group
    }
    graph, components, _edges, _strong = _overlap_graph(
        selected, recommended=recommended_overlap, cancel=cancel
    )
    bridge_attempts = 0
    while len(components) > 1 and bridge_attempts < 8:
        starting_component_count = len(components)
        if len(selected) >= target_budget:
            layer_totals = {}
            for candidate in selected:
                layer_totals[candidate.layer_name] = layer_totals.get(candidate.layer_name, 0) + 1
            largest = max(components, key=len)
            removable = [
                index for index in largest
                if layer_totals.get(selected[index].layer_name, 0) > 1
            ]
            if not removable:
                break
            victim_index = max(
                removable,
                key=lambda index: (
                    not selected[index].critical,
                    len(graph.get(index, ())),
                    selected[index].candidate_id,
                ),
            )
            victim = selected.pop(victim_index)
            selected_groups.discard(origin_group_indices.get(victim.origin_id))
            coverage = {}
            for candidate in selected:
                _apply_candidate(coverage, candidate, maximum_incidence)
            previous = selected[-1] if selected else None
            graph, components, _edges, _strong = _overlap_graph(
                selected, recommended=recommended_overlap, cancel=cancel
            )
            if len(components) <= 1:
                break
        component_cells = []
        for component in components:
            component_cells.append(set().union(*(_overlap_cell_keys(selected[index]) for index in component)))
        best = None
        for group_index, group in enumerate(groups):
            if group_index % 64 == 0 and cancel and cancel():
                raise PlanningCancelled("Scientific camera planning cancelled")
            if group_index in selected_groups:
                continue
            candidate = _best_from_group(
                cache, group, coverage, previous, minimum_obs, preferred_obs,
                selected=selected,
            )
            if candidate is None:
                continue
            touched = [len(_overlap_cell_keys(candidate) & cells) for cells in component_cells]
            positive = sorted((value for value in touched if value > 0), reverse=True)
            if len(positive) < 2:
                continue
            bridge_score = positive[0] + positive[1]
            key = (bridge_score, candidate.critical, candidate.score, -group_index, candidate.candidate_id)
            if best is None or key > best[0]:
                best = (key, group_index)
        if best is None or not add_group(best[1]):
            break
        selected[-1].kind = "bridge"
        bridge_added += 1
        graph, components, _edges, _strong = _overlap_graph(
            selected, recommended=recommended_overlap, cancel=cancel
        )
        bridge_attempts += 1
        if len(components) >= starting_component_count:
            break

    while len(selected) < target_budget:
        best = None
        for index, group in enumerate(groups):
            if index in selected_groups:
                continue
            candidate = _best_from_group(
                cache, group, coverage, previous, minimum_obs, preferred_obs,
                selected=selected,
            )
            if candidate is None:
                continue
            key = (candidate.score, candidate.is_base, -index, candidate.candidate_id)
            if best is None or key > best[0]:
                best = (key, index, candidate)
        if best is None:
            break
        if not add_group(best[1], forced_candidate=best[2]):
            break

    coverage = {}
    for candidate in selected:
        _apply_candidate(coverage, candidate, maximum_incidence)

    graph, components, edges, strong_neighbors = _overlap_graph(
        selected, recommended=recommended_overlap, cancel=cancel
    )
    return {
        "selected": selected,
        "selected_groups": selected_groups,
        "coverage": coverage,
        "target_budget": target_budget,
        "polar_added": polar_added,
        "bridge_added": bridge_added,
        "graph": graph,
        "components": components,
        "edges": edges,
        "strong_neighbors": strong_neighbors,
    }


def _coverage_statistics(
    cache, groups, selection, legacy_station_count, legacy_budget,
    stations_by_component, unsafe_count, visible=None, global_visible=None,
    path_visible=None, provider_visible=None,
):
    selected = selection["selected"]
    coverage = selection["coverage"]
    visible = visible if visible is not None else _all_visible_cells(groups)
    global_visible = global_visible if global_visible is not None else visible
    path_visible = path_visible if path_visible is not None else visible
    provider_visible = provider_visible if provider_visible is not None else visible
    counts = [coverage[key]["observation_count"] if key in coverage else 0 for key in visible]
    layer_counts = {}
    for candidate in selected:
        layer_counts[candidate.layer_name] = layer_counts.get(candidate.layer_name, 0) + 1
    origins = [_origin_key(candidate.position, 0.001 * cache.units_per_meter) for candidate in selected]
    duplicated = len(origins) - len(set(origins))
    steps = [station["step"] / cache.units_per_meter for stations in stations_by_component for station in stations]
    band_counts = {"NEAR": 0, "MID": 0, "FAR": 0, "REGULAR": 0}
    near_candidates = []
    for candidate in selected:
        band_counts[candidate.distance_band] = band_counts.get(candidate.distance_band, 0) + 1
        if candidate.distance_band == "NEAR":
            _count, overlap = _near_field_mid_neighbors(cache, candidate, selected)
            candidate.near_field_average_overlap = overlap
            candidate.shared_environment_ratio = overlap
            near_candidates.append(candidate)
    minimum_obs = int(cache.settings.scientific_minimum_observations)
    near_distance = float(getattr(
        cache.settings, "near_field_recommended_distance_min", 0.60
    )) * cache.units_per_meter
    near_surface_keys = {
        key
        for candidate in near_candidates
        for key, observation in candidate.observations.items()
        if observation.distance < near_distance
    }
    near_under_observed = sum(
        coverage.get(key, {}).get("observation_count", 0) < minimum_obs
        for key in near_surface_keys
    )

    def ratio(category):
        return _category_ratio(coverage, visible, category)

    per_object = {}
    for observation in visible.values():
        item = per_object.setdefault(observation.object_name, {"visible_surface_cells": 0, "observed_surface_cells": 0})
        item["visible_surface_cells"] += 1
    for key, observation in visible.items():
        if key in coverage:
            per_object[observation.object_name]["observed_surface_cells"] += 1
    for item in per_object.values():
        item["coverage_ratio"] = item["observed_surface_cells"] / max(1, item["visible_surface_cells"])
    space_counts = cache.origin_generation.get("space_class_counts", {})
    space_classification = {
        "normal_room_count": int(space_counts.get("NORMAL_ROOM", 0)),
        "small_room_count": int(space_counts.get("SMALL_ROOM", 0)),
        "corridor_count": int(space_counts.get("NARROW_CORRIDOR", 0)),
        "doorway_count": int(cache.origin_generation.get("doorway_node_count", 0)),
        "doorway_transition_count": int(space_counts.get("DOORWAY_TRANSITION", 0)),
        "cluttered_space_count": int(space_counts.get("CLUTTERED_SPACE", 0)),
    }
    global_observed = sum(key in coverage for key in global_visible)
    path_observed = sum(key in coverage for key in path_visible)
    provider_observed = sum(key in coverage for key in provider_visible)
    true_unreachable = len(set(cache.surface_cells) - set(global_visible))
    global_coverage = {
        "estimated_surface_cell_count": len(cache.surface_cells),
        "globally_reachable_surface_cell_count": len(global_visible),
        "provider_candidate_visible_surface_cell_count": len(provider_visible),
        "path_candidate_visible_surface_cell_count": len(path_visible),
        "final_observed_surface_cell_count": global_observed,
        "global_reachable_coverage_ratio": global_observed / max(1, len(global_visible)),
        "path_candidate_visible_coverage_ratio": path_observed / max(1, len(path_visible)),
        "provider_candidate_visible_coverage_ratio": provider_observed / max(1, len(provider_visible)),
        "true_unreachable_surface_cell_count": true_unreachable,
    }
    stats = {
        "mode": "SCIENTIFIC_THREE_LAYER",
        "origin_generation": dict(cache.origin_generation),
        "space_classification": space_classification,
        "global_coverage": global_coverage,
        "origin_mode": cache.origin_generation.get("mode", "CURVE"),
        "budget_mode": getattr(cache.settings, "scientific_budget_mode", "LEGACY_PATH_BUDGET"),
        "legacy_station_count": legacy_station_count,
        "legacy_view_budget": legacy_budget,
        "final_camera_count": len(selected),
        "layer_count": len(layer_counts),
        "layer_camera_counts": layer_counts,
        "near_field_camera_count": len(near_candidates),
        "rejected_too_close_candidate_count": len(cache.rejected_too_close_candidate_ids),
        "dominant_surface_rejected_count": len(cache.dominant_surface_rejected_ids),
        "near_field_quota_rejected_count": len(cache.near_field_quota_rejected_ids),
        "near_mid_far_camera_counts": {
            "near": band_counts.get("NEAR", 0),
            "mid": band_counts.get("MID", 0),
            "far": band_counts.get("FAR", 0),
            "regular": band_counts.get("REGULAR", 0),
        },
        "near_field_average_overlap": (
            sum(candidate.near_field_average_overlap for candidate in near_candidates)
            / len(near_candidates) if near_candidates else 0.0
        ),
        "near_field_under_observed_cells": near_under_observed,
        "target_overlap": float(cache.settings.scientific_target_overlap),
        "minimum_overlap": float(cache.settings.scientific_minimum_overlap),
        "minimum_step": float(cache.settings.scientific_minimum_step),
        "maximum_step": float(cache.settings.scientific_maximum_step),
        "actual_step_min": min(steps) if steps else 0.0,
        "actual_step_max": max(steps) if steps else 0.0,
        "actual_step_mean": sum(steps) / len(steps) if steps else 0.0,
        "duplicated_origin_count": duplicated,
        "coverage_cell_size": cache.coverage_cell_size / cache.units_per_meter,
        "visible_surface_cell_count": len(visible),
        "overall_visible_surface_coverage_ratio": sum(value >= 1 for value in counts) / max(1, len(visible)),
        "per_object_coverage": per_object,
        "cells_observed_at_least_1": sum(value >= 1 for value in counts),
        "cells_observed_at_least_3": sum(value >= 3 for value in counts),
        "cells_observed_at_least_5": sum(value >= 5 for value in counts),
        "floor_coverage_ratio": ratio("floor_like"),
        "ceiling_coverage_ratio": ratio("ceiling_like"),
        "vertical_surface_coverage_ratio": ratio("vertical_like"),
        "global_reachable_coverage_ratio": global_coverage["global_reachable_coverage_ratio"],
        "path_candidate_visible_coverage_ratio": global_coverage["path_candidate_visible_coverage_ratio"],
        "globally_reachable_surface_cell_count": len(global_visible),
        "true_unreachable_surface_cell_count": true_unreachable,
        "under_observed_cell_count": sum(value < minimum_obs for value in counts),
        "overlap_graph_component_count": len(selection["components"]),
        "polar_keyframe_count": sum(candidate.kind == "polar" for candidate in selected),
        "removed_redundant_camera_count": max(0, len(groups) - len(selected)),
        "bridge_camera_count": sum(candidate.kind == "bridge" for candidate in selected),
        "unreachable_surface_cells": true_unreachable,
        "estimated_surface_cell_count": len(cache.surface_cells),
        "unsafe_candidate_count": unsafe_count,
        "unsafe_candidate_reasons": {"clearance_not_satisfied": unsafe_count},
        "cameras_with_fewer_than_two_strong_neighbors": sum(value < 2 for value in selection["strong_neighbors"]),
        "horizontal_fov_degrees": math.degrees(cache.horizontal_fov),
        "vertical_fov_degrees": math.degrees(cache.vertical_fov),
        "ray_grid": [cache.ray_grid, cache.ray_grid],
        "prefilter_ray_grid": [cache.prefilter_ray_grid, cache.prefilter_ray_grid],
        "deterministic": True,
    }
    return stats


def prepare_scientific(scene, settings, components, component_counts, progress=None, cancel=None):
    origin_mode = getattr(settings, "scientific_origin_mode", "MANUAL_CURVE")
    needs_curve = origin_mode in {"MANUAL_CURVE", "AUTO_GRID_PATH"}
    if needs_curve and not components:
        return None
    cache = make_cache(scene, settings)
    legacy_station_count = sum(max(0, int(value)) for value in component_counts)
    path_budget = legacy_station_count * 6
    budget_mode = getattr(settings, "scientific_budget_mode", "LEGACY_PATH_BUDGET")
    minimum_budget = max(1, int(getattr(settings, "scientific_minimum_budget", 24)))
    maximum_budget = max(minimum_budget, int(getattr(settings, "scientific_maximum_budget", 3600)))
    if budget_mode == "AREA_ADAPTIVE_BUDGET" and cache.free_space_map is not None:
        computed_budget = round(cache.free_space_map.area_m2 * float(getattr(settings, "free_space_views_per_m2", 6.0)))
    elif budget_mode == "SURFACE_ADAPTIVE_BUDGET":
        computed_budget = round(math.sqrt(max(1, len(cache.surface_cells))) * 2.0)
    elif budget_mode == "USER_FIXED_BUDGET":
        computed_budget = int(getattr(settings, "scientific_fixed_budget", 720))
    else:
        computed_budget = path_budget or int(getattr(settings, "camera_count", 120))
    legacy_budget = int(_clamp(computed_budget, minimum_budget, maximum_budget))
    if progress:
        progress("path_sampling", 0, max(1, legacy_station_count))
    if origin_mode in {"FREE_SPACE", "SMALL_SPACE"}:
        provider = FreeSpaceOriginProvider(
            adaptive_small_spaces=(origin_mode == "SMALL_SPACE" or bool(getattr(settings, "scientific_auto_small_space", True)))
        )
    elif origin_mode == "HYBRID":
        provider = HybridOriginProvider(components, component_counts)
    else:
        provider = CurveOriginProvider(components, component_counts)
    groups, stations, unsafe_count = _make_candidate_groups(
        cache, components, component_counts, progress=progress, cancel=cancel,
        cast_rays=False, origin_provider=provider,
    )
    if not groups:
        return None
    if budget_mode == "AREA_ADAPTIVE_BUDGET" and cache.free_space_map is not None:
        computed_budget = round(
            cache.free_space_map.area_m2 * float(getattr(settings, "free_space_views_per_m2", 6.0))
            + len(cache.free_space_map.regions) * 6
        )
        legacy_budget = int(_clamp(computed_budget, minimum_budget, maximum_budget))
    return {
        "cache": cache,
        "groups": groups,
        "stations": stations,
        "unsafe_count": unsafe_count,
        "legacy_station_count": legacy_station_count,
        "legacy_budget": legacy_budget,
        "all_candidates": [candidate for group in groups for candidate in group],
    }


def cast_prepared_batch(state, start, count=4, progress=None, cancel=None):
    candidates = state["all_candidates"]
    end = min(len(candidates), start + max(1, int(count)))
    for index in range(start, end):
        if cancel and cancel():
            raise PlanningCancelled("Scientific camera planning cancelled")
        _cast_candidate(
            state["cache"], candidates[index], progress, cancel,
            ray_grid=state["cache"].prefilter_ray_grid,
        )
    if progress:
        progress("candidate_prefilter", end, max(1, len(candidates)))
    return end


def _visible_cells_for_provider(groups, provider_type):
    return _all_visible_cells([
        [candidate for candidate in group if candidate.provider_type == provider_type]
        for group in groups
        if any(candidate.provider_type == provider_type for candidate in group)
    ])


def _merge_visible_cells(*maps):
    result = {}
    for mapping in maps:
        for key, observation in mapping.items():
            old = result.get(key)
            if old is None or observation.incidence_angle < old.incidence_angle:
                result[key] = observation
    return result


def prepare_final_quality(state, progress=None, cancel=None):
    if state.get("selection") is not None:
        return state["selection"]
    cache = state["cache"]
    groups = state["groups"]
    provider_visible = _all_visible_cells(groups)
    path_visible = _visible_cells_for_provider(groups, "CURVE")
    origin_mode = cache.origin_generation.get("mode", "CURVE")
    global_enabled = origin_mode != "CURVE" or bool(
        getattr(cache.settings, "scientific_global_reachable_coverage", False)
    )
    global_visible = provider_visible
    if global_enabled and cache.free_space_map is None:
        global_provider = FreeSpaceOriginProvider(adaptive_small_spaces=True)
        global_provider.prepare(cache.scene, cache, cache.settings)
        global_provider.progress = progress
        global_provider.cancel = cancel
        global_provider.generate_origins()
        for group in global_provider.candidate_groups:
            for candidate in group:
                _cast_candidate(
                    cache, candidate, progress, cancel,
                    ray_grid=cache.prefilter_ray_grid,
                )
        state["global_reachable_groups"] = global_provider.candidate_groups
        global_visible = _merge_visible_cells(
            global_visible,
            _all_visible_cells(global_provider.candidate_groups),
        )
    analysis_selection = _select_candidates(
        cache, groups, state["legacy_budget"], progress=progress, cancel=cancel
    )
    coverage_enabled = bool(getattr(cache.settings, "scientific_coverage_driven", True)) and (
        origin_mode != "CURVE" or global_enabled
    )
    coverage_seeds = []
    if coverage_enabled and global_visible:
        coverage_provider = CoverageDrivenOriginProvider(
            global_visible,
            analysis_selection["coverage"],
            analysis_selection["selected"],
        )
        coverage_provider.prepare(cache.scene, cache, cache.settings)
        coverage_provider.progress = progress
        coverage_provider.cancel = cancel
        coverage_seeds = coverage_provider.generate_origins()
        for group in coverage_provider.candidate_groups:
            for candidate in group:
                _cast_candidate(
                    cache, candidate, progress, cancel,
                    ray_grid=cache.prefilter_ray_grid,
                )
        if coverage_provider.candidate_groups:
            groups.extend(coverage_provider.candidate_groups)
            reserve = min(
                len(coverage_seeds),
                max(1, int(round(state["legacy_budget"] * 0.15))),
            )
            cache.coverage_driven_reserve = reserve
            global_visible = _merge_visible_cells(
                global_visible,
                _all_visible_cells(coverage_provider.candidate_groups),
            )
            generation = cache.origin_generation
            providers = list(generation.get("providers", ()))
            if "COVERAGE_DRIVEN" not in providers:
                providers.append("COVERAGE_DRIVEN")
            generation["providers"] = providers
            counts = generation.setdefault("provider_origin_counts", {})
            counts["COVERAGE_DRIVEN"] = len(coverage_seeds)
            generation["generated_origin_count"] = generation.get("generated_origin_count", 0) + len(coverage_seeds) + coverage_provider.unsafe_count
            generation["valid_origin_count"] = generation.get("valid_origin_count", 0) + len(coverage_seeds)
            generation["coverage_driven_target_count"] = coverage_provider.target_count
            generation["coverage_driven_rejected_unsafe_count"] = coverage_provider.unsafe_count
    state["provider_visible"] = provider_visible
    state["path_visible"] = path_visible
    state["global_visible"] = global_visible
    state["prefilter_visible"] = global_visible if global_enabled else provider_visible
    selection = _select_candidates(
        cache, groups, state["legacy_budget"], progress=progress, cancel=cancel
    )
    cache.coverage_driven_reserve = 0
    state["selection"] = selection
    state["final_candidates"] = selection["selected"]
    state["final_quality_complete"] = False
    for candidate in state["final_candidates"]:
        cache.candidate_rays.pop(candidate.candidate_id, None)
        candidate.observations = {}
        candidate.overlap_cells.clear()
        candidate.ray_grid = 0
        candidate.quality_maps.clear()
    return selection


def cast_final_quality_batch(state, start, count=4, progress=None, cancel=None):
    prepare_final_quality(state, progress=progress, cancel=cancel)
    candidates = state["final_candidates"]
    end = min(len(candidates), start + max(1, int(count)))
    for index in range(start, end):
        if cancel and cancel():
            raise PlanningCancelled("Scientific camera planning cancelled")
        _cast_candidate(
            state["cache"], candidates[index], progress, cancel,
            ray_grid=state["cache"].ray_grid,
        )
    state["final_quality_complete"] = end >= len(candidates)
    if progress:
        progress("final_quality_rays", end, max(1, len(candidates)))
    return end


def _refresh_final_coverage(cache, selection):
    coverage = {}
    maximum_incidence = float(cache.settings.scientific_maximum_incidence_angle)
    for candidate in selection["selected"]:
        _apply_candidate(coverage, candidate, maximum_incidence)
    selection["coverage"] = coverage


def _refresh_final_selection(cache, selection, cancel=None):
    graph, components, edges, strong_neighbors = _overlap_graph(
        selection["selected"], recommended=float(cache.settings.scientific_minimum_overlap),
        cancel=cancel,
    )
    selection.update({
        "graph": graph,
        "components": components,
        "edges": edges,
        "strong_neighbors": strong_neighbors,
    })


def _enforce_final_near_field(cache, selection):
    if not bool(getattr(cache.settings, "near_field_protection", True)):
        return []
    selected = selection["selected"]
    if not any(candidate.provider_type != "CURVE" for candidate in selected):
        return []
    minimum_obs = int(cache.settings.scientific_minimum_observations)
    preferred_obs = max(
        minimum_obs, int(cache.settings.scientific_preferred_observations)
    )
    maximum_incidence = float(cache.settings.scientific_maximum_incidence_angle)
    rejected = set()
    near_ranked = []
    for index, candidate in enumerate(selected):
        if candidate.provider_type == "CURVE":
            continue
        others = [item for other_index, item in enumerate(selected) if other_index != index]
        coverage = {}
        for other in others:
            _apply_candidate(coverage, other, maximum_incidence)
        score = _score_candidate(
            cache,
            candidate,
            coverage,
            others[-1] if others else None,
            minimum_obs,
            preferred_obs,
            selected=others,
        )
        candidate.score = score
        if score <= NEAR_FIELD_REJECT_SCORE:
            rejected.add(index)
        elif candidate.distance_band == "NEAR":
            near_ranked.append((score, candidate.candidate_id, index, candidate))
    allowed_near = _near_field_camera_limit(cache.settings, len(selected))
    excess = max(0, len(near_ranked) - allowed_near)
    for _score, _candidate_id, index, candidate in sorted(near_ranked)[:excess]:
        rejected.add(index)
        candidate.near_field_rejected = True
        candidate.near_field_rejection_reason = "near_camera_quota"
        cache.near_field_quota_rejected_ids.add(candidate.candidate_id)
        cache.near_field_rejected_positions[candidate.candidate_id] = candidate.position.copy()
    removed_ids = [
        candidate.candidate_id
        for index, candidate in enumerate(selected)
        if index in rejected
    ]
    if rejected:
        selection["selected"] = [
            candidate for index, candidate in enumerate(selected)
            if index not in rejected
        ]
    return removed_ids


def _strong_neighbor_count(candidate, selected, skip_index, recommended):
    candidate_cells = _overlap_cell_keys(candidate)
    if not candidate_cells:
        return 0
    count = 0
    for other_index, other in enumerate(selected):
        if other_index == skip_index:
            continue
        other_cells = _overlap_cell_keys(other)
        if not other_cells:
            continue
        common = len(candidate_cells & other_cells)
        ratio = common / max(1, min(len(candidate_cells), len(other_cells)))
        baseline = (candidate.position - other.position).length
        useful_baseline = 0.10 * min(candidate.local_step, other.local_step)
        if ratio >= recommended and baseline >= useful_baseline:
            count += 1
    return count


def _repair_final_neighbors(cache, groups, selection, progress=None, cancel=None):
    origin_groups = {group[0].origin_id: group for group in groups if group}
    recommended = float(cache.settings.scientific_minimum_overlap)
    # Category and coverage repairs can create a short cascade of new deficits. Keep the
    # process bounded, but allow enough passes for those deterministic replacements to settle.
    for _repair_pass in range(8):
        _refresh_final_selection(cache, selection, cancel=cancel)
        current_neighbors = selection["strong_neighbors"]
        deficits = [index for index, count in enumerate(current_neighbors) if count < 2]
        if not deficits:
            return
        replacements = []
        for ordinal, index in enumerate(deficits):
            if cancel and cancel():
                raise PlanningCancelled("Scientific camera planning cancelled")
            if progress:
                progress("final_overlap_repair", ordinal + 1, len(deficits))
            current = selection["selected"][index]
            ranked = []
            for candidate in origin_groups.get(current.origin_id, ()):
                if candidate.ray_grid != cache.ray_grid:
                    cache.candidate_rays.pop(candidate.candidate_id, None)
                    candidate.observations = {}
                    candidate.overlap_cells.clear()
                    candidate.ray_grid = 0
                    candidate.quality_maps.clear()
                    _cast_candidate(cache, candidate, cancel=cancel, ray_grid=cache.ray_grid)
                neighbor_count = _strong_neighbor_count(
                    candidate, selection["selected"], index, recommended
                )
                ranked.append((
                    neighbor_count, len(candidate.observations), candidate.is_base,
                    -abs(candidate.pitch), candidate.candidate_id, candidate,
                ))
            if not ranked:
                continue
            best = max(ranked, key=lambda item: item[:-1])
            if best[0] > current_neighbors[index]:
                replacements.append((index, best[-1]))
        if not replacements:
            break
        for index, candidate in replacements:
            selection["selected"][index] = candidate
    _refresh_final_selection(cache, selection, cancel=cancel)


def _quality_cell_keys(cache, candidate):
    return _quality_cell_map(candidate, float(cache.settings.scientific_maximum_incidence_angle)).keys()


def _repair_final_categories(cache, groups, selection, visible, progress=None, cancel=None):
    if not getattr(cache.settings, "scientific_auto_floor_ceiling", True):
        return
    origin_groups = {group[0].origin_id: group for group in groups if group}
    visible_keys = set(visible)
    targets = (("floor_like", -1.0), ("ceiling_like", 1.0))
    recommended_overlap = float(cache.settings.scientific_minimum_overlap)
    cell_counts = {}
    for selected_candidate in selection["selected"]:
        for key in _quality_cell_keys(cache, selected_candidate):
            cell_counts[key] = cell_counts.get(key, 0) + 1
    for category, pitch_sign in targets:
        category_keys = {
            key for key, observation in visible.items() if observation.category == category
        }
        if not category_keys:
            continue
        for repair_index in range(8):
            covered_keys = {key for key, count in cell_counts.items() if count > 0}
            category_observed = len(covered_keys & category_keys)
            if category_observed / len(category_keys) >= 0.95:
                break
            overall_observed = len(covered_keys & visible_keys)
            best = None
            for index, current in enumerate(selection["selected"]):
                current_cells = _quality_cell_keys(cache, current)
                for candidate in origin_groups.get(current.origin_id, ()):
                    if candidate.kind != "polar" or candidate.pitch * pitch_sign <= 0.0:
                        continue
                    if candidate.ray_grid != cache.ray_grid:
                        if cancel and cancel():
                            raise PlanningCancelled("Scientific camera planning cancelled")
                        cache.candidate_rays.pop(candidate.candidate_id, None)
                        candidate.observations = {}
                        candidate.overlap_cells.clear()
                        candidate.ray_grid = 0
                        candidate.quality_maps.clear()
                        _cast_candidate(cache, candidate, cancel=cancel, ray_grid=cache.ray_grid)
                    neighbor_count = _strong_neighbor_count(
                        candidate, selection["selected"], index, recommended_overlap
                    )
                    if neighbor_count < 2:
                        continue
                    candidate_cells = _quality_cell_keys(cache, candidate)
                    category_gain = len((candidate_cells & category_keys) - covered_keys)
                    category_loss = sum(
                        key in category_keys and cell_counts.get(key, 0) <= 1
                        and key not in candidate_cells for key in current_cells
                    )
                    overall_gain = len((candidate_cells & visible_keys) - covered_keys)
                    overall_loss = sum(
                        key in visible_keys and cell_counts.get(key, 0) <= 1
                        and key not in candidate_cells for key in current_cells
                    )
                    projected_overall = overall_observed + overall_gain - overall_loss
                    current_ratio = overall_observed / max(1, len(visible_keys))
                    projected_ratio = projected_overall / max(1, len(visible_keys))
                    if current_ratio >= 0.95 and projected_ratio < 0.95:
                        continue
                    if current_ratio < 0.95 and projected_overall < overall_observed:
                        continue
                    key = (
                        category_gain - category_loss, category_gain,
                        overall_gain - overall_loss, neighbor_count,
                        candidate.candidate_id,
                    )
                    if key[0] > 0 and (best is None or key > best[0]):
                        best = (key, index, candidate)
            if best is None:
                break
            old_candidate = selection["selected"][best[1]]
            for key in _quality_cell_keys(cache, old_candidate):
                cell_counts[key] -= 1
            for key in _quality_cell_keys(cache, best[2]):
                cell_counts[key] = cell_counts.get(key, 0) + 1
            selection["selected"][best[1]] = best[2]
            if progress:
                progress("final_category_repair", repair_index + 1, 8)


def _repair_final_overall(cache, groups, selection, visible, progress=None, cancel=None):
    if not getattr(cache.settings, "scientific_auto_coverage", True):
        return
    origin_groups = {group[0].origin_id: group for group in groups if group}
    visible_keys = set(visible)
    if not visible_keys:
        return
    cell_counts = {}
    for selected_candidate in selection["selected"]:
        for key in _quality_cell_keys(cache, selected_candidate):
            cell_counts[key] = cell_counts.get(key, 0) + 1
    for repair_index in range(8):
        covered_keys = {key for key, count in cell_counts.items() if count > 0}
        if len(covered_keys & visible_keys) / len(visible_keys) >= 0.95:
            break
        shortlist = []
        for index, current in enumerate(selection["selected"]):
            for candidate in origin_groups.get(current.origin_id, ()):
                if candidate is current or candidate.kind != "normal":
                    continue
                approximate_gain = sum(
                    key in visible_keys and key not in covered_keys
                    for key in candidate.observations
                )
                if approximate_gain:
                    shortlist.append((
                        approximate_gain, candidate.is_base,
                        candidate.candidate_id, index, candidate,
                    ))
        if not shortlist:
            break
        best = None
        for _approximate, _base, _candidate_id, index, candidate in sorted(
            shortlist, key=lambda item: item[:-1], reverse=True
        )[:32]:
            if cancel and cancel():
                raise PlanningCancelled("Scientific camera planning cancelled")
            if candidate.ray_grid != cache.ray_grid:
                cache.candidate_rays.pop(candidate.candidate_id, None)
                candidate.observations = {}
                candidate.overlap_cells.clear()
                candidate.ray_grid = 0
                candidate.quality_maps.clear()
                _cast_candidate(cache, candidate, cancel=cancel, ray_grid=cache.ray_grid)
            current = selection["selected"][index]
            current_cells = _quality_cell_keys(cache, current)
            candidate_cells = _quality_cell_keys(cache, candidate)
            gain = len((candidate_cells & visible_keys) - covered_keys)
            loss = sum(
                key in visible_keys and cell_counts.get(key, 0) <= 1
                and key not in candidate_cells for key in current_cells
            )
            key = (gain - loss, gain, candidate.is_base, candidate.candidate_id)
            if key[0] > 0 and (best is None or key > best[0]):
                best = (key, index, candidate)
        if best is None:
            break
        old_candidate = selection["selected"][best[1]]
        for key in _quality_cell_keys(cache, old_candidate):
            cell_counts[key] -= 1
        for key in _quality_cell_keys(cache, best[2]):
            cell_counts[key] = cell_counts.get(key, 0) + 1
        selection["selected"][best[1]] = best[2]
        if progress:
            progress("final_coverage_repair", repair_index + 1, 8)


def finalize_scientific(state, progress=None, cancel=None):
    cache = state["cache"]
    groups = state["groups"]
    selection = prepare_final_quality(state, progress=progress, cancel=cancel)
    if not state.get("final_quality_complete", False):
        index = 0
        while index < len(state["final_candidates"]):
            index = cast_final_quality_batch(
                state, index, count=16, progress=progress, cancel=cancel
            )
    _repair_final_overall(
        cache, groups, selection, state["prefilter_visible"], progress=progress, cancel=cancel
    )
    _repair_final_categories(
        cache, groups, selection, state["prefilter_visible"], progress=progress, cancel=cancel
    )
    _repair_final_neighbors(cache, groups, selection, progress=progress, cancel=cancel)
    near_field_removed = _enforce_final_near_field(cache, selection)
    # Intermediate repair stages maintain their own compact cell counts. Build the full
    # observation report once, after every deterministic replacement has settled.
    _refresh_final_coverage(cache, selection)
    _refresh_final_selection(cache, selection, cancel=cancel)
    stats = _coverage_statistics(
        cache, groups, selection, state["legacy_station_count"],
        state["legacy_budget"], state["stations"], state["unsafe_count"],
        visible=state["prefilter_visible"],
        global_visible=state.get("global_visible"),
        path_visible=state.get("path_visible"),
        provider_visible=state.get("provider_visible"),
    )
    stats["near_field_final_removed_count"] = len(near_field_removed)
    stats["near_field_removed_candidate_ids"] = near_field_removed
    return {
        "cache": cache, "groups": groups, "stations": state["stations"],
        "selected": selection["selected"], "coverage": selection["coverage"],
        "graph": selection["graph"], "edges": selection["edges"],
        "visible": state["prefilter_visible"], "stats": stats,
        "global_visible": state.get("global_visible", state["prefilter_visible"]),
        "path_visible": state.get("path_visible", {}),
        "provider_visible": state.get("provider_visible", state["prefilter_visible"]),
    }


def plan_scientific(scene, settings, components, component_counts, progress=None, cancel=None):
    state = prepare_scientific(
        scene, settings, components, component_counts, progress=progress, cancel=cancel
    )
    if state is None:
        return None
    index = 0
    while index < len(state["all_candidates"]):
        index = cast_prepared_batch(state, index, count=16, progress=progress, cancel=cancel)
    prepare_final_quality(state, progress=progress, cancel=cancel)
    index = 0
    while index < len(state["final_candidates"]):
        index = cast_final_quality_batch(state, index, count=16, progress=progress, cancel=cancel)
    return finalize_scientific(state, progress=progress, cancel=cancel)


def recast_final_cameras(plan, cameras, affected_candidate_ids=(), cancel=None):
    """Make coverage and overlap statistics match the actual render camera poses."""
    try:
        bpy.context.view_layer.update()
    except Exception:
        pass
    cache = plan["cache"]
    affected = set(affected_candidate_ids)
    by_id = {candidate.candidate_id: candidate for candidate in plan["selected"]}
    surviving = []
    recast_count = 0
    for camera in cameras:
        candidate_id = str(camera.get("gs_scientific_candidate_id", ""))
        candidate = by_id.get(candidate_id)
        if candidate is None:
            continue
        if candidate_id in affected:
            forward = (
                camera.matrix_world.to_quaternion() @ Vector((0.0, 0.0, -1.0))
            ).normalized()
            candidate.position = camera.matrix_world.translation.copy()
            candidate.yaw = math.atan2(forward.y, forward.x) % math.tau
            candidate.pitch = math.asin(_clamp(forward.z, -1.0, 1.0))
            candidate.tangent = forward.copy()
            cache.candidate_rays.pop(candidate.candidate_id, None)
            candidate.observations = {}
            candidate.overlap_cells.clear()
            candidate.quality_maps.clear()
            candidate.ray_grid = 0
            _cast_candidate(cache, candidate, cancel=cancel, ray_grid=cache.ray_grid)
            recast_count += 1
        surviving.append(candidate)

    old_stats = dict(plan["stats"])
    selection = {"selected": surviving}
    near_field_removed = _enforce_final_near_field(cache, selection)
    surviving = selection["selected"]
    coverage_before = float(
        old_stats.get(
            "global_reachable_coverage_ratio",
            old_stats.get("overall_visible_surface_coverage_ratio", 0.0),
        )
    )
    _refresh_final_coverage(cache, selection)
    _refresh_final_selection(cache, selection, cancel=cancel)
    refreshed = _coverage_statistics(
        cache,
        plan["groups"],
        selection,
        int(old_stats.get("legacy_station_count", 0)),
        int(old_stats.get("legacy_view_budget", len(surviving))),
        plan.get("stations", ()),
        int(old_stats.get("unsafe_candidate_count", 0)),
        visible=plan.get("visible"),
        global_visible=plan.get("global_visible"),
        path_visible=plan.get("path_visible"),
        provider_visible=plan.get("provider_visible"),
    )
    for key, value in old_stats.items():
        refreshed.setdefault(key, value)
    coverage_after = float(
        refreshed.get(
            "global_reachable_coverage_ratio",
            refreshed.get("overall_visible_surface_coverage_ratio", 0.0),
        )
    )
    refreshed.update({
        "post_clipping_recast_camera_count": recast_count,
        "post_clipping_coverage_before": coverage_before,
        "post_clipping_coverage_after": coverage_after,
        "post_clipping_repair_camera_count": 0,
        "final_camera_count": len(surviving),
        "near_field_post_clipping_removed_count": len(near_field_removed),
        "near_field_removed_candidate_ids": near_field_removed,
    })
    plan["selected"] = surviving
    plan["coverage"] = selection["coverage"]
    plan["graph"] = selection["graph"]
    plan["edges"] = selection["edges"]
    plan["stats"].clear()
    plan["stats"].update(refreshed)
    return refreshed


def camera_matrix(position, yaw, pitch):
    """Return the authoritative Blender camera matrix for a scientific candidate."""
    forward, right, _up = _direction_basis(yaw, pitch)
    z_axis = -forward
    x_axis = right
    y_axis = z_axis.cross(x_axis).normalized()
    rotation = Matrix((
        (x_axis.x, y_axis.x, z_axis.x),
        (x_axis.y, y_axis.y, z_axis.y),
        (x_axis.z, y_axis.z, z_axis.z),
    ))
    return Matrix.Translation(Vector(position)) @ rotation.to_4x4()


def orient_camera(camera, yaw, pitch):
    camera.matrix_world = camera_matrix(camera.location, yaw, pitch)


def _debug_collection(scene):
    old = bpy.data.collections.get(DEBUG_COLLECTION)
    if old is not None:
        for obj in list(old.objects):
            if not obj.get("gs_path_planner_debug"):
                bpy.data.objects.remove(obj, do_unlink=True)
        return old
    collection = bpy.data.collections.new(DEBUG_COLLECTION)
    scene.collection.children.link(collection)
    return collection


def clear_debug_display(scene):
    old = bpy.data.collections.get(DEBUG_COLLECTION)
    if old is None:
        return
    for obj in list(old.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for parent in list(scene.collection.children):
        if parent == old:
            scene.collection.children.unlink(old)
    bpy.data.collections.remove(old)


def _debug_curve(collection, name, points, color):
    if len(points) < 2:
        return None
    data = bpy.data.curves.new(name, "CURVE")
    data.dimensions = "3D"
    data.bevel_depth = 0.006
    data.bevel_resolution = 1
    spline = data.splines.new("POLY")
    spline.points.add(len(points) - 1)
    for target, point in zip(spline.points, points):
        target.co = (*point, 1.0)
    obj = bpy.data.objects.new(name, data)
    obj.color = color
    obj.hide_render = True
    obj["gs_scientific_debug"] = True
    collection.objects.link(obj)
    return obj


def _debug_points(collection, name, points, color):
    if not points:
        return None
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(points, [], [])
    obj = bpy.data.objects.new(name, mesh)
    obj.color = color
    obj.display_type = "WIRE"
    obj.hide_render = True
    obj["gs_scientific_debug"] = True
    collection.objects.link(obj)
    return obj


def build_debug_display(scene, settings, plan, cameras):
    collection = _debug_collection(scene)
    selected = plan["selected"]
    free_map = plan["cache"].free_space_map
    if free_map is not None:
        _debug_points(
            collection,
            "GS_FreeSpace_Legal",
            [Vector((cell.x, cell.y, cell.floor_z + 0.02 * plan["cache"].units_per_meter)) for cell in free_map.cells.values()],
            (0.10, 0.85, 0.30, 1.0),
        )
        _debug_points(
            collection,
            "GS_FreeSpace_Occupied",
            free_map.invalid_points,
            (0.85, 0.12, 0.08, 1.0),
        )
        _debug_points(
            collection,
            "GS_FreeSpace_MedialAxis",
            [Vector((cell.x, cell.y, cell.floor_z + 0.05 * plan["cache"].units_per_meter)) for cell in free_map.medial_axis_nodes],
            (0.05, 0.85, 0.95, 1.0),
        )
        _debug_points(
            collection,
            "GS_FreeSpace_Doorways",
            [Vector((cell.x, cell.y, cell.floor_z + 0.08 * plan["cache"].units_per_meter)) for cell in free_map.doorway_nodes],
            (1.0, 0.65, 0.05, 1.0),
        )
        _debug_points(
            collection,
            "GS_Provider_Origins",
            [group[0].position for group in plan["groups"] if group],
            (0.95, 0.45, 0.10, 1.0),
        )
    band_colors = {
        "FAR": (0.10, 0.55, 1.0, 1.0),
        "MID": (0.15, 0.90, 0.35, 1.0),
        "NEAR": (0.95, 0.25, 0.80, 1.0),
    }
    for band in ("FAR", "MID", "NEAR"):
        _debug_points(
            collection,
            f"GS_NearField_{band}_Cameras",
            [candidate.position for candidate in selected if candidate.distance_band == band],
            band_colors[band],
        )
    rejected_positions = plan["cache"].near_field_rejected_positions
    _debug_points(
        collection,
        "GS_NearField_Rejected_TooClose",
        [rejected_positions[candidate_id]
         for candidate_id in sorted(plan["cache"].rejected_too_close_candidate_ids)
         if candidate_id in rejected_positions],
        (1.0, 0.05, 0.02, 1.0),
    )
    _debug_points(
        collection,
        "GS_NearField_Rejected_DominantSurface",
        [rejected_positions[candidate_id]
         for candidate_id in sorted(plan["cache"].dominant_surface_rejected_ids)
         if candidate_id in rejected_positions],
        (1.0, 0.45, 0.02, 1.0),
    )
    layer_colors = {
        "Low": (0.12, 0.55, 1.0, 1.0),
        "Middle": (0.2, 0.9, 0.35, 1.0),
        "High": (1.0, 0.7, 0.1, 1.0),
    }
    grouped = {}
    for candidate in selected:
        grouped.setdefault((candidate.component_index, candidate.layer_name), []).append(candidate)
    for (component_index, layer_name), candidates in grouped.items():
        candidates.sort(key=lambda item: item.distance_along)
        color = layer_colors.get(layer_name, (0.7, 0.35, 1.0, 1.0))
        _debug_curve(collection, f"GS_Debug_Path_{component_index:03d}_{layer_name}", [item.position for item in candidates], color)

    if getattr(settings, "scientific_keep_candidates", False):
        selected_origins = {candidate.origin_id for candidate in selected}
        for group in plan["groups"]:
            representative = group[0]
            if representative.origin_id in selected_origins:
                continue
            obj = bpy.data.objects.new(f"GS_Removed_{representative.origin_id}", None)
            obj.empty_display_type = "ARROWS"
            obj.empty_display_size = 0.08 * plan["cache"].units_per_meter
            obj.location = representative.position
            obj.color = (0.35, 0.35, 0.35, 1.0)
            obj["gs_scientific_debug"] = True
            collection.objects.link(obj)

    minimum_observations = int(settings.scientific_minimum_observations)
    under_points = []
    for key, observation in plan.get("visible", {}).items():
        entry = plan["coverage"].get(key)
        if entry is None:
            under_points.append(observation.position)
        elif entry["observation_count"] < minimum_observations:
            under_points.append(entry["position"])
    if not plan.get("visible"):
        under_points.extend(
            entry["position"] for entry in plan["coverage"].values()
            if entry["observation_count"] < minimum_observations
        )
    if under_points:
        mesh = bpy.data.meshes.new("GS_Undercovered_Surface_Markers")
        mesh.from_pydata(under_points, [], [])
        obj = bpy.data.objects.new("GS_Undercovered_Surface_Markers", mesh)
        obj.color = (1.0, 0.05, 0.02, 1.0)
        obj.display_type = "WIRE"
        obj.hide_render = True
        obj["gs_scientific_debug"] = True
        collection.objects.link(obj)

    if getattr(settings, "scientific_show_overlap_graph", True):
        for edge_index, (left, right, ratio) in enumerate(plan["edges"]):
            if ratio < 0.10:
                continue
            _debug_curve(
                collection,
                f"GS_Overlap_{edge_index:05d}",
                [selected[left].position, selected[right].position],
                (0.1, 0.8, 0.9, 1.0),
            )

    provider_colors = {
        "CURVE": (0.15, 0.65, 1.0, 1.0),
        "FREE_SPACE": (0.95, 0.45, 0.10, 1.0),
        "COVERAGE_DRIVEN": (0.95, 0.25, 0.90, 1.0),
    }
    for camera, candidate in zip(cameras, selected):
        if hasattr(camera, "color"):
            camera.color = (
            band_colors.get(candidate.distance_band)
            if candidate.provider_type != "CURVE"
            else None
        ) or provider_colors.get(
            candidate.provider_type, (0.55, 0.80, 0.25, 1.0)
        )
    return collection
