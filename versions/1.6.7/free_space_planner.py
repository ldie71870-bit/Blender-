"""Deterministic 2.5D free-space analysis for scientific camera origins."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from mathutils import Vector


@dataclass
class FreeSpaceCell:
    key: tuple
    x: float
    y: float
    floor_z: float
    ceiling_z: float
    clearance: float
    component_id: int = -1
    boundary: bool = False
    doorway: bool = False
    medial: bool = False


@dataclass
class SpaceRegion:
    region_id: str
    cell_keys: list
    area_m2: float
    minimum_width_m: float
    mean_clearance_m: float
    clearance_p10_m: float
    clearance_p50_m: float
    aspect_ratio: float
    safe_origin_count: int
    doorway_count: int
    furniture_occupancy_ratio: float
    classification: str
    center: Vector
    principal_axis: Vector
    cross_axis: Vector
    near_field: bool = False


@dataclass
class FreeSpaceOrigin:
    cell: FreeSpaceCell
    source: str
    preferred_direction: Vector
    critical: bool = False
    distance_band: str = "REGULAR"
    view_role: str = "general"


@dataclass
class FreeSpaceMap:
    resolution: float
    resolution_m: float
    cells: dict
    regions: list
    invalid_cell_count: int
    boundary_mask: set = field(default_factory=set)
    doorway_nodes: list = field(default_factory=list)
    medial_axis_nodes: list = field(default_factory=list)
    candidate_origins: list = field(default_factory=list)
    invalid_points: list = field(default_factory=list)

    @property
    def area_m2(self):
        return len(self.cells) * self.resolution_m * self.resolution_m


def _clamp(value, low, high):
    return max(low, min(high, value))


def _percentile(values, fraction):
    values = sorted(values)
    if not values:
        return 0.0
    position = _clamp(fraction, 0.0, 1.0) * (len(values) - 1)
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return values[low]
    return values[low] + (values[high] - values[low]) * (position - low)


def _horizontal_clearance(cache, ray_cast, point, maximum):
    nearest = maximum
    for index in range(16):
        angle = math.tau * index / 16.0
        direction = Vector((math.cos(angle), math.sin(angle), 0.0))
        hit, location, _normal, _face, _obj = ray_cast(
            cache, point + direction * 1e-4, direction, maximum
        )
        if hit:
            nearest = min(nearest, (location - point).length)
    return nearest


def _grid_resolution(cache, settings):
    requested_m = float(getattr(settings, "free_space_grid_resolution", 0.35))
    requested_m = _clamp(requested_m, 0.10, 1.00)
    width_m = max(0.01, (cache.scene_max.x - cache.scene_min.x) / cache.units_per_meter)
    depth_m = max(0.01, (cache.scene_max.y - cache.scene_min.y) / cache.units_per_meter)
    maximum = max(1000, int(getattr(settings, "free_space_max_grid_cells", 200000)))
    required_m = math.sqrt(width_m * depth_m / maximum)
    resolution_m = _clamp(max(requested_m, required_m), 0.10, 2.00)
    return resolution_m * cache.units_per_meter, resolution_m


def _components(cells, floor_tolerance):
    remaining = set(cells)
    result = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        stack = [seed]
        component = []
        while stack:
            key = stack.pop()
            component.append(key)
            floor_z = cells[key].floor_z
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (key[0] + dx, key[1] + dy)
                if neighbor not in remaining:
                    continue
                if abs(cells[neighbor].floor_z - floor_z) > floor_tolerance:
                    continue
                remaining.remove(neighbor)
                stack.append(neighbor)
        result.append(sorted(component))
    return sorted(result, key=lambda keys: (-len(keys), keys[0]))


def _region_axes(keys, cells):
    points = [Vector((cells[key].x, cells[key].y, 0.0)) for key in keys]
    center = sum(points, Vector()) / max(1, len(points))
    xx = sum((point.x - center.x) ** 2 for point in points) / max(1, len(points))
    yy = sum((point.y - center.y) ** 2 for point in points) / max(1, len(points))
    xy = sum((point.x - center.x) * (point.y - center.y) for point in points) / max(1, len(points))
    angle = 0.5 * math.atan2(2.0 * xy, xx - yy) if abs(xy) + abs(xx - yy) > 1e-12 else 0.0
    main = Vector((math.cos(angle), math.sin(angle), 0.0))
    cross = Vector((-main.y, main.x, 0.0))
    return center, main, cross


def _mark_structure(cells, component_keys):
    clearances = [cells[key].clearance for key in component_keys]
    median = _percentile(clearances, 0.50)
    low_clearance = _percentile(clearances, 0.35)
    component_set = set(component_keys)
    doorway_nodes = []
    medial_nodes = []
    boundary = set()
    for key in component_keys:
        cell = cells[key]
        neighbors = [
            (key[0] + dx, key[1] + dy)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
        ]
        valid_neighbors = [neighbor for neighbor in neighbors if neighbor in component_set]
        cell.boundary = len(valid_neighbors) < 4
        if cell.boundary:
            boundary.add(key)
        neighbor_clearances = [cells[neighbor].clearance for neighbor in valid_neighbors]
        cell.medial = bool(
            neighbor_clearances
            and cell.clearance >= median
            and cell.clearance >= max(neighbor_clearances) - 1e-6
        )
        if cell.medial:
            medial_nodes.append(cell)
        east = (key[0] + 1, key[1])
        west = (key[0] - 1, key[1])
        north = (key[0], key[1] + 1)
        south = (key[0], key[1] - 1)
        opposing_x = east in component_set and west in component_set
        opposing_y = north in component_set and south in component_set
        wider_x = bool(
            opposing_x
            and min(cells[east].clearance, cells[west].clearance) >= cell.clearance * 1.25
        )
        wider_y = bool(
            opposing_y
            and min(cells[north].clearance, cells[south].clearance) >= cell.clearance * 1.25
        )
        cell.doorway = bool(
            cell.clearance <= max(low_clearance, median * 0.75)
            and (wider_x or wider_y)
        )
        if cell.doorway:
            doorway_nodes.append(cell)
    return boundary, doorway_nodes, medial_nodes


def _cluster_doorway_nodes(nodes):
    by_key = {cell.key: cell for cell in nodes}
    remaining = set(by_key)
    representatives = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        stack = [seed]
        cluster = []
        while stack:
            key = stack.pop()
            cluster.append(by_key[key])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbor = (key[0] + dx, key[1] + dy)
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        stack.append(neighbor)
        representatives.append(min(cluster, key=lambda cell: (cell.clearance, cell.key)))
    for cell in nodes:
        cell.doorway = False
    for cell in representatives:
        cell.doorway = True
    return representatives


def _classify_region(
    region_id, keys, cells, resolution_m, units_per_meter,
    invalid_cell_count, near_field_distance_m,
):
    clearances_m = [cells[key].clearance for key in keys]
    center, main, cross = _region_axes(keys, cells)
    main_values = [
        (Vector((cells[key].x, cells[key].y, 0.0)) - center).dot(main)
        for key in keys
    ]
    cross_values = [
        (Vector((cells[key].x, cells[key].y, 0.0)) - center).dot(cross)
        for key in keys
    ]
    main_span_m = (max(main_values) - min(main_values)) / max(1e-9, units_per_meter)
    cross_span_m = (max(cross_values) - min(cross_values)) / max(1e-9, units_per_meter)
    shorter = max(resolution_m, min(main_span_m, cross_span_m))
    longer = max(main_span_m, cross_span_m, shorter)
    aspect = longer / shorter
    area_m2 = len(keys) * resolution_m * resolution_m
    p10 = _percentile(clearances_m, 0.10) / units_per_meter
    p50 = _percentile(clearances_m, 0.50) / units_per_meter
    mean = sum(clearances_m) / max(1, len(clearances_m)) / units_per_meter
    minimum_width = max(resolution_m, 2.0 * p10)
    doorway_count = sum(cells[key].doorway for key in keys)
    xs = [key[0] for key in keys]
    ys = [key[1] for key in keys]
    box_cells = max(1, (max(xs) - min(xs) + 1) * (max(ys) - min(ys) + 1))
    occupancy = _clamp(1.0 - len(keys) / box_cells, 0.0, 1.0)
    safe_origins = sum(cells[key].clearance / units_per_meter >= 0.20 for key in keys)
    if area_m2 <= 2.0 and doorway_count:
        classification = "DOORWAY_TRANSITION"
    elif aspect >= 2.5 and (minimum_width <= 2.2 or p50 <= 1.0):
        classification = "NARROW_CORRIDOR"
    elif area_m2 <= 8.0 or minimum_width <= 1.35 or safe_origins < 6:
        classification = "SMALL_ROOM"
    elif occupancy >= 0.35 or mean <= 0.55:
        classification = "CLUTTERED_SPACE"
    else:
        classification = "NORMAL_ROOM"
    near_field = bool(
        p50 < near_field_distance_m
        or minimum_width < near_field_distance_m * 2.0
    )
    return SpaceRegion(
        region_id=region_id,
        cell_keys=list(keys),
        area_m2=area_m2,
        minimum_width_m=minimum_width,
        mean_clearance_m=mean,
        clearance_p10_m=p10,
        clearance_p50_m=p50,
        aspect_ratio=aspect,
        safe_origin_count=safe_origins,
        doorway_count=doorway_count,
        furniture_occupancy_ratio=occupancy,
        classification=classification,
        center=center,
        principal_axis=main,
        cross_axis=cross,
        near_field=near_field,
    )


def build_free_space_map(cache, settings, ray_cast, cancel=None, progress=None):
    resolution, resolution_m = _grid_resolution(cache, settings)
    min_x, min_y = cache.scene_min.x, cache.scene_min.y
    max_x, max_y = cache.scene_max.x, cache.scene_max.y
    nx = max(1, int(math.ceil((max_x - min_x) / resolution)))
    ny = max(1, int(math.ceil((max_y - min_y) / resolution)))
    probe_height = float(getattr(settings, "free_space_probe_height", 1.40)) * cache.units_per_meter
    minimum_headroom = float(getattr(settings, "free_space_min_headroom", 1.20)) * cache.units_per_meter
    minimum_clearance = min(
        float(getattr(settings, "scientific_camera_clearance", 0.25)),
        float(getattr(settings, "free_space_narrow_clearance", 0.12)),
    ) * cache.units_per_meter
    horizontal_search = max(3.0 * cache.units_per_meter, resolution * 6.0)
    vertical_search = cache.scene_diagonal * 1.5
    cells = {}
    invalid = 0
    invalid_points = []
    total = (nx + 1) * (ny + 1)
    for iy in range(ny + 1):
        if cancel and cancel():
            raise RuntimeError("Scientific camera planning cancelled")
        y = min(max_y, min_y + iy * resolution)
        for ix in range(nx + 1):
            index = iy * (nx + 1) + ix
            if progress and (index % 256 == 0 or index + 1 == total):
                progress("free_space_grid", index + 1, total)
            x = min(max_x, min_x + ix * resolution)
            probe_z = _clamp(
                cache.scene_min.z + probe_height,
                cache.scene_min.z + resolution,
                cache.scene_max.z - resolution,
            )
            probe = Vector((x, y, probe_z))
            floor_hit, floor_location, *_ = ray_cast(
                cache, probe, Vector((0.0, 0.0, -1.0)), vertical_search
            )
            ceiling_hit, ceiling_location, *_ = ray_cast(
                cache, probe, Vector((0.0, 0.0, 1.0)), vertical_search
            )
            if not floor_hit or not ceiling_hit:
                invalid += 1
                if len(invalid_points) < 20000:
                    invalid_points.append(probe.copy())
                continue
            if ceiling_location.z - floor_location.z < minimum_headroom:
                invalid += 1
                if len(invalid_points) < 20000:
                    invalid_points.append(probe.copy())
                continue
            camera_z = _clamp(
                floor_location.z + probe_height,
                floor_location.z + minimum_clearance,
                ceiling_location.z - minimum_clearance,
            )
            camera_point = Vector((x, y, camera_z))
            clearance = _horizontal_clearance(
                cache, ray_cast, camera_point, horizontal_search
            )
            if clearance < minimum_clearance:
                invalid += 1
                if len(invalid_points) < 20000:
                    invalid_points.append(camera_point.copy())
                continue
            cells[(ix, iy)] = FreeSpaceCell(
                key=(ix, iy),
                x=x,
                y=y,
                floor_z=floor_location.z,
                ceiling_z=ceiling_location.z,
                clearance=clearance,
            )

    components = _components(cells, 0.25 * cache.units_per_meter)
    regions = []
    boundary_mask = set()
    doorway_nodes = []
    medial_nodes = []
    for component_id, keys in enumerate(components):
        for key in keys:
            cells[key].component_id = component_id
        boundary, doorways, medial = _mark_structure(cells, keys)
        doorways = _cluster_doorway_nodes(doorways)
        boundary_mask.update(boundary)
        doorway_nodes.extend(doorways)
        medial_nodes.extend(medial)
        regions.append(
            _classify_region(
                f"free:{component_id}",
                keys,
                cells,
                resolution_m,
                cache.units_per_meter,
                invalid,
                float(getattr(settings, "near_field_recommended_distance_min", 0.60)),
            )
        )
    return FreeSpaceMap(
        resolution=resolution,
        resolution_m=resolution_m,
        cells=cells,
        regions=regions,
        invalid_cell_count=invalid,
        boundary_mask=boundary_mask,
        doorway_nodes=doorway_nodes,
        medial_axis_nodes=medial_nodes,
        invalid_points=invalid_points,
    )


def _preferred_direction(cell, region, cells):
    point = Vector((cell.x, cell.y, 0.0))
    to_center = region.center - point
    if cell.doorway:
        east = cells.get((cell.key[0] + 1, cell.key[1]))
        west = cells.get((cell.key[0] - 1, cell.key[1]))
        north = cells.get((cell.key[0], cell.key[1] + 1))
        south = cells.get((cell.key[0], cell.key[1] - 1))
        x_value = (east.clearance if east else 0.0) + (west.clearance if west else 0.0)
        y_value = (north.clearance if north else 0.0) + (south.clearance if south else 0.0)
        direction = Vector((1.0, 0.0, 0.0)) if x_value >= y_value else Vector((0.0, 1.0, 0.0))
        return direction
    if to_center.length > 1e-6:
        return to_center.normalized()
    return region.principal_axis.copy()


def _farthest_cells(pool, initial, count, spacing):
    if count <= len(initial):
        return initial[:count]
    selected = list(initial)
    selected_keys = {cell.key for cell in selected}
    if not selected and pool:
        selected.append(max(pool, key=lambda cell: (cell.clearance, -cell.key[1], -cell.key[0])))
        selected_keys.add(selected[0].key)
    minimum_distances = {}
    spacing_squared = spacing * spacing
    for cell in pool:
        if cell.key in selected_keys:
            continue
        minimum_distances[cell.key] = min(
            (cell.x - chosen.x) ** 2 + (cell.y - chosen.y) ** 2
            for chosen in selected
        )
    while len(selected) < count and minimum_distances:
        key = max(
            minimum_distances,
            key=lambda item: (
                minimum_distances[item],
                pool_by_key[item].clearance,
                -item[1],
                -item[0],
            ),
        )
        if minimum_distances[key] < spacing_squared and selected:
            break
        chosen = pool_by_key[key]
        selected.append(chosen)
        del minimum_distances[key]
        for other_key in list(minimum_distances):
            other = pool_by_key[other_key]
            distance = (other.x - chosen.x) ** 2 + (other.y - chosen.y) ** 2
            minimum_distances[other_key] = min(minimum_distances[other_key], distance)
    return selected


def _nearest_region_cell(cells, target, excluded=()):
    excluded = set(excluded)
    available = [cell for cell in cells if cell.key not in excluded]
    if not available:
        return None
    return min(
        available,
        key=lambda cell: (
            (cell.x - target.x) ** 2 + (cell.y - target.y) ** 2,
            -cell.clearance,
            cell.key,
        ),
    )


def _doorway_tier_origins(region, cells, all_cells, resolution, units_per_meter):
    result = []
    maximum_projection = max(resolution * 2.50, 0.85 * units_per_meter)
    for doorway in sorted((cell for cell in cells if cell.doorway), key=lambda cell: cell.key):
        axis = _preferred_direction(doorway, region, all_cells).normalized()
        doorway_point = Vector((doorway.x, doorway.y, 0.0))
        if (region.center - doorway_point).dot(axis) < 0.0:
            axis.negate()
        cross = Vector((-axis.y, axis.x, 0.0))
        specifications = (
            ("FAR", "door_outside_in", -1.20, 0.0, axis),
            ("MID", "doorway_bridge", -0.65, 0.0, axis),
            ("MID", "door_inside_out", 0.65, 0.0, -axis),
            ("FAR", "adjacent_space_backshot", 1.20, 0.0, -axis),
            ("MID", "door_left_offset", -0.20, -0.40, axis),
            ("MID", "door_right_offset", -0.20, 0.40, axis),
            ("NEAR", "door_detail", 0.0, 0.0, axis),
        )
        used_keys = set()
        for band, role, along_m, cross_m, direction in specifications:
            target = (
                doorway_point
                + axis * along_m * units_per_meter
                + cross * cross_m * units_per_meter
            )
            cell = _nearest_region_cell(cells, target, excluded=used_keys)
            if cell is None:
                continue
            distance = math.hypot(cell.x - target.x, cell.y - target.y)
            if distance > maximum_projection:
                continue
            used_keys.add(cell.key)
            result.append(FreeSpaceOrigin(
                cell=cell,
                source="doorway_tier",
                preferred_direction=direction.copy(),
                critical=True,
                distance_band=band,
                view_role=role,
            ))
    priority = {"FAR": 0, "MID": 1, "NEAR": 2, "REGULAR": 3}
    result.sort(key=lambda item: (
        priority.get(item.distance_band, 3),
        item.cell.key,
        item.view_role,
    ))
    return result


pool_by_key = {}


def generate_candidate_origins(free_map, settings, units_per_meter):
    global pool_by_key
    maximum = max(1, int(getattr(settings, "free_space_max_origin_count", 600)))
    layers = max(1, int(getattr(settings, "scientific_layer_count", 3)))
    planar_maximum = max(1, maximum // layers)
    spacing = max(
        free_map.resolution,
        float(getattr(settings, "free_space_candidate_spacing", 0.75)) * units_per_meter,
    )
    boundary_bias = _clamp(
        float(getattr(settings, "free_space_boundary_bias", 0.25)), 0.0, 1.0
    )
    doorway_priority = bool(getattr(settings, "free_space_doorway_priority", True))
    medial_priority = bool(getattr(settings, "free_space_medial_axis_priority", True))
    recommended_min = float(getattr(settings, "near_field_recommended_distance_min", 0.60))
    recommended_max = float(getattr(settings, "near_field_recommended_distance_max", 1.00))
    origins = []
    for region in free_map.regions:
        cells = [free_map.cells[key] for key in region.cell_keys]
        if not cells:
            continue
        area_share = region.area_m2 / max(1e-9, free_map.area_m2)
        quota = max(1, int(round(planar_maximum * area_share)))
        quota = min(quota, len(cells))
        doorway_tiers = _doorway_tier_origins(
            region, cells, free_map.cells, free_map.resolution, units_per_meter
        ) if doorway_priority else []
        quota = min(
            len(cells), max(quota, min(len(doorway_tiers), planar_maximum))
        )
        special_by_key = {origin.cell.key: origin for origin in doorway_tiers}
        initial = [origin.cell for origin in doorway_tiers]
        initial.append(max(cells, key=lambda cell: (cell.clearance, -cell.key[1], -cell.key[0])))
        if doorway_priority:
            initial.extend(sorted(
                (cell for cell in cells if cell.doorway),
                key=lambda cell: (-cell.clearance, cell.key),
            )[:max(1, quota // 5)])
        if medial_priority:
            initial.extend(sorted(
                (cell for cell in cells if cell.medial),
                key=lambda cell: (-cell.clearance, cell.key),
            )[:max(1, quota // 3)])
        if boundary_bias > 0.0:
            initial.extend(sorted(
                (cell for cell in cells if cell.boundary),
                key=lambda cell: (-cell.clearance, cell.key),
            )[:int(round(quota * boundary_bias))])
        deduped = []
        seen = set()
        for cell in initial:
            if cell.key not in seen:
                seen.add(cell.key)
                deduped.append(cell)
        pool = cells
        if len(pool) > 20000:
            stride = int(math.ceil(len(pool) / 20000))
            pool = pool[::stride]
        pool_by_key = {cell.key: cell for cell in pool}
        chosen = _farthest_cells(pool, deduped, quota, spacing)
        for cell in chosen:
            special = special_by_key.get(cell.key)
            if special is not None:
                origins.append(special)
                continue
            if cell.doorway:
                source = "doorway"
            elif cell.medial:
                source = "medial_axis"
            elif cell.boundary:
                source = "boundary"
            elif cell is initial[0]:
                source = "maximum_clearance"
            else:
                source = "farthest_point"
            clearance_m = cell.clearance / max(1e-9, units_per_meter)
            if clearance_m < recommended_min:
                distance_band = "NEAR"
            elif clearance_m <= recommended_max * 1.25:
                distance_band = "MID"
            else:
                distance_band = "FAR"
            origins.append(FreeSpaceOrigin(
                cell=cell,
                source=source,
                preferred_direction=_preferred_direction(cell, region, free_map.cells),
                critical=cell.doorway or source == "maximum_clearance",
                distance_band=distance_band,
                view_role="near_field_regular" if distance_band == "NEAR" else "general",
            ))
    origins.sort(key=lambda item: (
        item.cell.component_id,
        {"FAR": 0, "MID": 1, "REGULAR": 1, "NEAR": 2}.get(item.distance_band, 1),
        item.cell.key,
        item.source,
    ))
    free_map.candidate_origins = origins[:planar_maximum]
    return free_map.candidate_origins


def nearest_cells(free_map, position, maximum=8):
    """Return deterministic nearest legal planar cells for coverage-driven projection."""
    ranked = sorted(
        free_map.cells.values(),
        key=lambda cell: (
            (cell.x - position.x) ** 2 + (cell.y - position.y) ** 2,
            -cell.clearance,
            cell.key,
        ),
    )
    return ranked[:max(1, int(maximum))]

