"""Small dependency-free 3D walkable-path planner.

The Blender adapter supplies walkable samples and collision validation.  This
module deliberately contains no bpy/mathutils imports so its topology, stitching
and sampling rules can be regression-tested without launching Blender.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field


Point = tuple[float, float, float]


@dataclass(frozen=True)
class WalkableCell:
    key: tuple
    point: Point
    floor_z: float
    clearance: float
    surface_id: str = ""
    portal_candidate: bool = False


@dataclass
class FloorRegion:
    region_id: str
    cell_keys: list[tuple]
    mean_floor_z: float
    area_m2: float
    connector_ids: list[str] = field(default_factory=list)


@dataclass
class Connector:
    connector_id: str
    kind: str
    cell_keys: list[tuple]
    region_ids: tuple[str, ...]
    points: list[Point]


@dataclass
class SpaceRegion:
    region_id: str
    parent_floor_id: str
    kind: str
    cell_keys: list[tuple]
    area_m2: float


@dataclass
class Portal:
    portal_id: str
    floor_region_id: str
    space_region_ids: tuple[str, str]
    cell_keys: list[tuple]
    points: list[Point]


@dataclass
class PathFragment:
    fragment_id: str
    points: list[Point]
    region_ids: tuple[str, ...]
    kind: str = "CENTERLINE"
    connector_id: str = ""


@dataclass(frozen=True)
class PlannerConfig:
    grid_spacing: float
    units_per_meter: float = 1.0
    floor_tolerance_m: float = 0.10
    maximum_connector_step_m: float = 0.38
    minimum_floor_cells: int = 4
    minimum_floor_area_m2: float = 2.0
    stitch_distance_m: float = 1.25
    stitch_angle_degrees: float = 65.0
    smoothing_iterations: int = 2
    large_room_area_m2: float = 28.0
    coverage_path_count: int = 2
    minimum_connector_rise_m: float = 0.55
    minimum_connector_lane_ratio: float = 3.0
    major_floor_area_m2: float = 8.0
    room_minimum_area_m2: float = 4.0
    room_minimum_width_m: float = 1.8
    room_minimum_lanes: int = 2
    coverage_radius_m: float = 1.25
    coverage_target_ratio: float = 0.82


@dataclass(frozen=True)
class ArcSample:
    distance: float
    point: Point
    tangent: Point
    curvature: float
    critical: bool
    on_connector: bool


@dataclass
class PlanResult:
    regions: list[FloorRegion]
    connectors: list[Connector]
    raw_fragments: list[PathFragment]
    final_fragments: list[PathFragment]
    graph: dict
    stats: dict
    space_regions: list[SpaceRegion] = field(default_factory=list)
    portals: list[Portal] = field(default_factory=list)


def _point(value) -> Point:
    return (float(value[0]), float(value[1]), float(value[2]))


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(a, value):
    return (a[0] * value, a[1] * value, a[2] * value)


def _length(a):
    return math.sqrt(sum(value * value for value in a))


def _distance(a, b):
    return _length(_sub(a, b))


def _normalize(a):
    length = _length(a)
    return (1.0, 0.0, 0.0) if length <= 1e-12 else _mul(a, 1.0 / length)


def _dot(a, b):
    return sum(left * right for left, right in zip(a, b))


def _lerp(a, b, factor):
    return _add(a, _mul(_sub(b, a), factor))


def _angle(a, b):
    return math.acos(max(-1.0, min(1.0, _dot(_normalize(a), _normalize(b)))))


def _cell(value) -> WalkableCell:
    if isinstance(value, WalkableCell):
        return value
    key = tuple(value["key"] if isinstance(value, dict) else value.key)
    if hasattr(value, "point"):
        point = _point(value.point)
    elif isinstance(value, dict) and "point" in value:
        point = _point(value["point"])
    else:
        point = (float(value.x), float(value.y), float(value.floor_z))
    floor_z = float(value.get("floor_z", point[2]) if isinstance(value, dict) else getattr(value, "floor_z", point[2]))
    clearance = float(value.get("clearance", 1.0) if isinstance(value, dict) else getattr(value, "clearance", 1.0))
    surface_id = str(value.get("surface_id", "") if isinstance(value, dict) else getattr(value, "surface_id", ""))
    portal_candidate = bool(
        value.get("portal_candidate", False)
        if isinstance(value, dict) else getattr(value, "portal_candidate", False)
    )
    return WalkableCell(key, point, floor_z, clearance, surface_id, portal_candidate)


def _components(graph, keys=None):
    remaining = set(graph if keys is None else keys)
    result = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        stack = [seed]
        component = []
        while stack:
            item = stack.pop()
            component.append(item)
            for neighbor, _cost in graph.get(item, ()):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
        result.append(sorted(component))
    return result


def build_walkable_graph(cells, config: PlannerConfig, edge_validator=None):
    """Connect XY-neighbour samples while preserving every independent Z layer."""
    validator = edge_validator or (lambda _left, _right: True)
    by_key = {_cell(item).key: _cell(item) for item in cells}
    by_xy = {}
    for cell in by_key.values():
        by_xy.setdefault((cell.key[0], cell.key[1]), []).append(cell)
    graph = {key: [] for key in by_key}
    maximum_rise = config.maximum_connector_step_m * config.units_per_meter
    # Two-cell visibility bridges recover a doorway when coarse raster samples
    # miss the narrow threshold strip.  Blender's edge validator still checks
    # the actual segment and floor support, so walls and voids cannot be jumped.
    maximum_horizontal = config.grid_spacing * 2.30
    directions = (
        (1, 0), (0, 1), (1, 1), (1, -1),
        (2, 0), (0, 2), (2, 1), (1, 2), (2, -1), (1, -2),
    )
    for cell in by_key.values():
        ix, iy = cell.key[:2]
        for dx, dy in directions:
            for other in by_xy.get((ix + dx, iy + dy), ()):
                horizontal = math.hypot(other.point[0] - cell.point[0], other.point[1] - cell.point[1])
                rise = abs(other.floor_z - cell.floor_z)
                if horizontal <= 1e-8 or horizontal > maximum_horizontal or rise > maximum_rise:
                    continue
                if not validator(cell.point, other.point):
                    continue
                cost = _distance(cell.point, other.point)
                graph[cell.key].append((other.key, cost))
                graph[other.key].append((cell.key, cost))
    for key in graph:
        graph[key].sort(key=lambda item: item[0])
    return by_key, graph


def reachable_cell_keys(graph, seed_key):
    """Return exactly the 3D graph island reachable from ``seed_key``."""
    seed_key = tuple(seed_key) if seed_key is not None else None
    if seed_key not in graph:
        return set()
    reachable = {seed_key}
    stack = [seed_key]
    while stack:
        key = stack.pop()
        for neighbor, _cost in graph.get(key, ()):
            if neighbor not in reachable:
                reachable.add(neighbor)
                stack.append(neighbor)
    return reachable


def _restrict_graph(cells, graph, allowed):
    allowed = set(allowed)
    return (
        {key: cells[key] for key in allowed},
        {
            key: [(neighbor, cost) for neighbor, cost in graph[key] if neighbor in allowed]
            for key in allowed
        },
    )


def _apply_distance_field(cells, graph, config):
    """Fold reachable-mask boundary distance into clearance for centre bias."""
    if not cells:
        return cells, 0.0
    # A complete 8-neighbour interior sample has eight links.  Samples next to
    # walls, holes, disconnected space or an eroded floor edge are boundaries.
    boundary = [
        key for key in cells
        if sum(
            abs(neighbor[0] - key[0]) <= 1 and abs(neighbor[1] - key[1]) <= 1
            for neighbor, _cost in graph.get(key, ())
        ) < 8
    ]
    if not boundary:
        boundary = [min(cells)]
    distances = {key: 0.0 for key in boundary}
    queue = [(0.0, key) for key in boundary]
    heapq.heapify(queue)
    while queue:
        distance, key = heapq.heappop(queue)
        if distance != distances.get(key):
            continue
        for neighbor, cost in graph.get(key, ()):
            candidate = distance + cost
            if candidate + 1e-12 < distances.get(neighbor, float("inf")):
                distances[neighbor] = candidate
                heapq.heappush(queue, (candidate, neighbor))
    half_cell = config.grid_spacing * 0.5
    result = {}
    for key, cell in cells.items():
        mask_clearance = distances.get(key, 0.0) + half_cell
        result[key] = WalkableCell(
            cell.key, cell.point, cell.floor_z,
            min(cell.clearance, mask_clearance), cell.surface_id, cell.portal_candidate,
        )
    maximum = max(distances.values(), default=0.0) + half_cell
    return result, maximum


def _walkable_mask_validator(cells, config):
    """Build a final-geometry validator for the already restricted 3D mask."""
    spacing = max(config.grid_spacing, 1e-9)
    buckets = {}
    for cell in cells.values():
        bucket = (math.floor(cell.point[0] / spacing), math.floor(cell.point[1] / spacing))
        buckets.setdefault(bucket, []).append(cell)
    horizontal_limit = spacing * 0.86
    vertical_limit = max(
        config.maximum_connector_step_m * config.units_per_meter,
        spacing * 0.86,
    )

    def contains(point):
        bx = math.floor(point[0] / spacing)
        by = math.floor(point[1] / spacing)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for cell in buckets.get((bx + dx, by + dy), ()):
                    if (
                        math.hypot(point[0] - cell.point[0], point[1] - cell.point[1])
                        <= horizontal_limit
                        and abs(point[2] - cell.point[2]) <= vertical_limit
                    ):
                        return True
        return False

    def segment(left, right):
        distance = _distance(left, right)
        steps = max(1, int(math.ceil(distance / max(spacing * 0.30, 1e-9))))
        return all(contains(_lerp(left, right, index / steps)) for index in range(steps + 1))

    return segment


def _flat_graph(cells, graph, tolerance):
    result = {key: [] for key in graph}
    for key, neighbors in graph.items():
        for neighbor, cost in neighbors:
            left_surface = cells[key].surface_id
            right_surface = cells[neighbor].surface_id
            same_surface = not left_surface or not right_surface or left_surface == right_surface
            if same_surface and abs(cells[key].floor_z - cells[neighbor].floor_z) <= tolerance:
                result[key].append((neighbor, cost))
    return result


def identify_floor_regions(cells, graph, config):
    tolerance = config.floor_tolerance_m * config.units_per_meter
    flat = _flat_graph(cells, graph, tolerance)
    candidates = _components(flat)
    cell_area = (config.grid_spacing / max(1e-9, config.units_per_meter)) ** 2
    candidates = [
        keys for keys in candidates
        if len(keys) >= config.minimum_floor_cells
        and len(keys) * cell_area >= config.minimum_floor_area_m2
    ]
    candidates.sort(key=lambda keys: (sum(cells[key].floor_z for key in keys) / len(keys), min(keys)))
    regions = []
    membership = {}
    for index, keys in enumerate(candidates):
        region_id = f"Floor_{index}"
        for key in keys:
            membership[key] = region_id
        regions.append(FloorRegion(
            region_id=region_id,
            cell_keys=keys,
            mean_floor_z=sum(cells[key].floor_z for key in keys) / len(keys),
            area_m2=len(keys) * cell_area,
        ))
    return regions, membership


def _dijkstra(graph, start, allowed=None):
    allowed = set(graph) if allowed is None else set(allowed)
    distances = {start: 0.0}
    previous = {}
    queue = [(0.0, start)]
    while queue:
        distance, key = heapq.heappop(queue)
        if distance != distances.get(key):
            continue
        for neighbor, cost in graph.get(key, ()):
            if neighbor not in allowed:
                continue
            candidate = distance + cost
            if candidate + 1e-12 < distances.get(neighbor, float("inf")):
                distances[neighbor] = candidate
                previous[neighbor] = key
                heapq.heappush(queue, (candidate, neighbor))
    return distances, previous


def _path(previous, start, end):
    if end != start and end not in previous:
        return []
    result = [end]
    while result[-1] != start:
        result.append(previous[result[-1]])
    result.reverse()
    return result


def identify_connectors(cells, graph, regions, membership, config):
    tolerance = config.floor_tolerance_m * config.units_per_meter
    steep_nodes = set()
    for key, neighbors in graph.items():
        for neighbor, _cost in neighbors:
            if abs(cells[key].floor_z - cells[neighbor].floor_z) > tolerance:
                steep_nodes.update((key, neighbor))
    # Keep the short flat tread/landing samples between rising edges.  Large
    # flat components are already claimed by FloorRegion and are included only
    # at a rising boundary, so a same-XY upper/lower floor can never leak in.
    connector_nodes = set(steep_nodes)
    connector_nodes.update(key for key in graph if key not in membership)
    connector_graph = {key: [] for key in connector_nodes}
    for key in connector_nodes:
        for neighbor, cost in graph[key]:
            if neighbor in connector_nodes:
                connector_graph[key].append((neighbor, cost))
    connectors = []
    region_by_id = {region.region_id: region for region in regions}
    for keys in _components(connector_graph):
        if not (set(keys) & steep_nodes):
            continue
        adjacent_regions = set()
        for key in keys:
            if key in membership:
                adjacent_regions.add(membership[key])
            for neighbor, _cost in graph[key]:
                if neighbor in membership:
                    adjacent_regions.add(membership[neighbor])
        if len(adjacent_regions) < 2:
            continue
        low = min(keys, key=lambda key: (cells[key].floor_z, key))
        high = max(keys, key=lambda key: (cells[key].floor_z, key))
        _distances, previous = _dijkstra(connector_graph, low, allowed=keys)
        ordered = _path(previous, low, high) or sorted(keys, key=lambda key: (cells[key].floor_z, key))
        horizontal = sum(math.hypot(
            cells[right].point[0] - cells[left].point[0],
            cells[right].point[1] - cells[left].point[1],
        ) for left, right in zip(ordered, ordered[1:]))
        rise = abs(cells[high].floor_z - cells[low].floor_z)
        regional_rise = max(region_by_id[rid].mean_floor_z for rid in adjacent_regions) - min(
            region_by_id[rid].mean_floor_z for rid in adjacent_regions
        )
        minimum_rise = config.minimum_connector_rise_m * config.units_per_meter
        if rise < minimum_rise or regional_rise < minimum_rise:
            continue
        vertical_backtrack = sum(
            max(0.0, cells[left].floor_z - cells[right].floor_z)
            for left, right in zip(ordered, ordered[1:])
        )
        if vertical_backtrack > tolerance:
            continue
        lane_ratio = len(keys) / max(1, len(ordered))
        major_region_count = sum(
            region_by_id[rid].area_m2 >= config.major_floor_area_m2
            for rid in adjacent_regions
        )
        if lane_ratio + 1e-9 < config.minimum_connector_lane_ratio and major_region_count < 2:
            continue
        grade = rise / max(horizontal, 1e-9)
        kind = "RAMP" if grade <= 0.20 else "STAIR"
        connector_id = f"{kind.title()}_{len(connectors)}"
        region_ids = tuple(sorted(adjacent_regions, key=lambda rid: region_by_id[rid].mean_floor_z))
        connector = Connector(connector_id, kind, keys, region_ids, [cells[key].point for key in ordered])
        connectors.append(connector)
        for region_id in region_ids:
            region_by_id[region_id].connector_ids.append(connector_id)
    return connectors


def _space_kind(keys, cells, config):
    """Classify a reachable component with deliberately simple geometry rules."""
    if not keys:
        return "CORRIDOR"
    xs = [cells[key].point[0] for key in keys]
    ys = [cells[key].point[1] for key in keys]
    cell_m = config.grid_spacing / max(config.units_per_meter, 1e-9)
    width_m = (min(max(xs) - min(xs), max(ys) - min(ys)) / max(config.units_per_meter, 1e-9)) + cell_m
    area_m2 = len(keys) * cell_m * cell_m
    return "ROOM" if (
        area_m2 >= config.room_minimum_area_m2
        and width_m >= config.room_minimum_width_m
    ) else "CORRIDOR"


def classify_reachable_space(cells, graph, regions, config):
    """Split floors into rooms/corridors and retain narrow necks as portals.

    The split is derived from the reachable-mask distance field, not mesh object
    names.  Low-clearance boundary cells are assigned to the nearest open core;
    only a narrow interface between two cores becomes a portal.  Broad
    interfaces are merged back so an unobstructed large room is not fragmented.
    """
    space_regions = []
    portals = []
    units = max(config.units_per_meter, 1e-9)
    cell_area = (config.grid_spacing / units) ** 2
    # A room core must be wider than an ordinary residential doorway.  The
    # low-clearance band between such cores is then classified as PORTAL.
    core_clearance = max(config.grid_spacing * 1.50, 0.90 * units)
    for floor in regions:
        allowed = set(floor.cell_keys)
        floor_graph = {
            key: [
                (neighbor, cost) for neighbor, cost in graph.get(key, ())
                if neighbor in allowed
                and abs(neighbor[0] - key[0]) <= 1
                and abs(neighbor[1] - key[1]) <= 1
            ]
            for key in allowed
        }
        core = {
            key for key in allowed
            if cells[key].clearance + 1e-9 >= core_clearance
            and not cells[key].portal_candidate
        }
        core_components = [component for component in _components(floor_graph, core) if len(component) >= 2]
        if len(core_components) < 2:
            region_id = f"{floor.region_id}_Space_0"
            space_regions.append(SpaceRegion(
                region_id, floor.region_id, _space_kind(floor.cell_keys, cells, config),
                list(floor.cell_keys), len(floor.cell_keys) * cell_area,
            ))
            continue

        # Multi-source shortest assignment grows every open core through its
        # surrounding boundary band.  Deterministic tie-breaking keeps output
        # stable across Blender sessions.
        distances = {}
        labels = {}
        queue = []
        for label, component in enumerate(core_components):
            for key in component:
                distances[key] = 0.0
                labels[key] = label
                heapq.heappush(queue, (0.0, label, key))
        while queue:
            distance, label, key = heapq.heappop(queue)
            if distance != distances.get(key) or label != labels.get(key):
                continue
            for neighbor, cost in floor_graph.get(key, ()):
                candidate = distance + cost
                current = distances.get(neighbor, float("inf"))
                if candidate + 1e-12 < current or (
                    abs(candidate - current) <= 1e-12 and label < labels.get(neighbor, label + 1)
                ):
                    distances[neighbor] = candidate
                    labels[neighbor] = label
                    heapq.heappush(queue, (candidate, label, neighbor))

        interfaces = {}
        for key in allowed:
            for neighbor, _cost in floor_graph[key]:
                left, right = labels.get(key), labels.get(neighbor)
                if left is None or right is None or left == right:
                    continue
                pair = tuple(sorted((left, right)))
                interfaces.setdefault(pair, set()).update((key, neighbor))

        # Union broad interfaces: a doorway is a neck, while an artificial
        # distance-field seam across an open room has many interface samples.
        parent = list(range(len(core_components)))

        def find(value):
            while parent[value] != value:
                parent[value] = parent[parent[value]]
                value = parent[value]
            return value

        def union(left, right):
            left, right = find(left), find(right)
            if left != right:
                parent[max(left, right)] = min(left, right)

        broad_limit = max(
            10,
            int(math.ceil(2.25 * units / max(config.grid_spacing, 1e-9))) * 4,
        )
        for pair, interface_keys in interfaces.items():
            if len(interface_keys) >= broad_limit:
                union(*pair)
        labels = {key: find(label) for key, label in labels.items()}

        grouped = {}
        for key in allowed:
            grouped.setdefault(labels.get(key, 0), []).append(key)
        id_by_label = {}
        for index, (label, keys) in enumerate(sorted(grouped.items())):
            region_id = f"{floor.region_id}_Space_{index}"
            id_by_label[label] = region_id
            space_regions.append(SpaceRegion(
                region_id, floor.region_id, _space_kind(keys, cells, config),
                sorted(keys), len(keys) * cell_area,
            ))

        final_interfaces = {}
        for key in allowed:
            for neighbor, _cost in floor_graph[key]:
                left, right = labels.get(key), labels.get(neighbor)
                if left is None or right is None or left == right:
                    continue
                pair = tuple(sorted((left, right)))
                final_interfaces.setdefault(pair, set()).update((key, neighbor))
        for pair, interface_keys in sorted(final_interfaces.items()):
            # Choose the shortest in-mask route between both sides.  Explicit
            # portal candidates win ties and make real doorway sampling robust.
            left_keys = [key for key in interface_keys if labels.get(key) == pair[0]]
            right_keys = [key for key in interface_keys if labels.get(key) == pair[1]]
            if not left_keys or not right_keys:
                continue
            best = min(
                ((0 if (cells[a].portal_candidate or cells[b].portal_candidate) else 1,
                  _distance(cells[a].point, cells[b].point), a, b)
                 for a in left_keys for b in right_keys
                 if any(n == b for n, _c in floor_graph.get(a, ()))),
                default=None,
            )
            if best is None:
                continue
            _preferred, _gap, left_key, right_key = best
            portal_id = f"Portal_{len(portals)}"
            portals.append(Portal(
                portal_id, floor.region_id,
                (id_by_label[pair[0]], id_by_label[pair[1]]),
                sorted(interface_keys),
                [cells[left_key].point, cells[right_key].point],
            ))
    return space_regions, portals


def _reachable_region_ids(cells, graph, regions, membership, connectors, seed_key):
    if not regions or seed_key not in graph:
        return set()
    seed_region = membership.get(seed_key)
    if seed_region is None:
        distances, _previous = _dijkstra(graph, seed_key)
        candidates = [key for key in membership if key in distances]
        if candidates:
            nearest = min(candidates, key=lambda key: (distances[key], key))
            seed_region = membership[nearest]
    if seed_region is None:
        return set()
    adjacency = {region.region_id: set() for region in regions}
    for connector in connectors:
        for region_id in connector.region_ids:
            adjacency.setdefault(region_id, set()).update(
                other for other in connector.region_ids if other != region_id
            )
    reachable = {seed_region}
    stack = [seed_region]
    while stack:
        region_id = stack.pop()
        for neighbor in adjacency.get(region_id, ()):
            if neighbor not in reachable:
                reachable.add(neighbor)
                stack.append(neighbor)
    return reachable


def _weighted_region_graph(cells, graph, keys):
    allowed = set(keys)
    maximum = max((cells[key].clearance for key in allowed), default=1.0)
    result = {key: [] for key in allowed}
    for key in allowed:
        for neighbor, cost in graph[key]:
            if neighbor not in allowed:
                continue
            clearance = min(cells[key].clearance, cells[neighbor].clearance)
            penalty = 1.0 + 1.5 * max(0.0, maximum - clearance) / max(maximum, 1e-9)
            result[key].append((neighbor, cost * penalty))
    return result


def _diameter_keys(graph, keys, preferred=()):
    keys = set(keys)
    if not keys:
        return []
    candidates = [key for key in keys if len([n for n, _c in graph[key] if n in keys]) <= 2]
    candidates = sorted(set(candidates) | (set(preferred) & keys)) or sorted(keys)
    preferred = sorted(set(preferred) & keys)
    if preferred:
        start = preferred[0]
        if len(preferred) > 1:
            best = None
            for candidate_start in preferred:
                distances, previous = _dijkstra(graph, candidate_start, keys)
                for candidate_end in preferred:
                    pair = (distances.get(candidate_end, -1.0), candidate_start, candidate_end, previous)
                    if best is None or pair[:3] > best[:3]:
                        best = pair
            if best and best[0] >= 0.0:
                return _path(best[3], best[1], best[2])
        distances, previous = _dijkstra(graph, start, keys)
        end = max(candidates, key=lambda key: (distances.get(key, -1.0), key))
        return _path(previous, start, end) or [start]
    start = candidates[0]
    distances, _previous = _dijkstra(graph, start, keys)
    start = max(candidates, key=lambda key: (distances.get(key, -1.0), key))
    distances, previous = _dijkstra(graph, start, keys)
    end = max(candidates, key=lambda key: (distances.get(key, -1.0), key))
    return _path(previous, start, end) or [start]


def _coverage_fragments(region, cells, config, start_index):
    if region.area_m2 < config.large_room_area_m2 or config.coverage_path_count <= 0:
        return []
    keys = region.cell_keys
    xs = [cells[key].point[0] for key in keys]
    ys = [cells[key].point[1] for key in keys]
    along_x = (max(xs) - min(xs)) >= (max(ys) - min(ys))
    bucket_axis = 1 if along_x else 0
    along_axis = 0 if along_x else 1
    buckets = {}
    for key in keys:
        bucket = key[1] if along_x else key[0]
        buckets.setdefault(bucket, []).append(key)
    ordered_buckets = sorted(buckets)
    result = []
    for number in range(config.coverage_path_count):
        fraction = (number + 1) / (config.coverage_path_count + 1)
        bucket = ordered_buckets[int(round(fraction * (len(ordered_buckets) - 1)))]
        row = sorted(buckets[bucket], key=lambda key: cells[key].point[along_axis])
        if len(row) < 2:
            continue
        result.append(PathFragment(
            f"fragment_{start_index + len(result):04d}",
            [cells[key].point for key in row],
            (region.region_id,),
            "COVERAGE_OFFSET",
        ))
    return result


def _space_lane_candidates(space_region, cells, graph, config):
    keys = list(space_region.cell_keys)
    if len(keys) < 2:
        return []
    xs = [cells[key].point[0] for key in keys]
    ys = [cells[key].point[1] for key in keys]
    along_x = (max(xs) - min(xs)) >= (max(ys) - min(ys))
    bucket_axis = 1 if along_x else 0
    along_axis = 0 if along_x else 1
    buckets = {}
    for key in keys:
        coordinate = cells[key].point[bucket_axis]
        bucket = int(round(coordinate / max(config.grid_spacing, 1e-9)))
        buckets.setdefault(bucket, []).append(key)
    candidates = []
    allowed = set(keys)
    for bucket, row_keys in sorted(buckets.items()):
        ordered = sorted(row_keys, key=lambda key: cells[key].point[along_axis])
        run = []
        for key in ordered:
            if run:
                previous = run[-1]
                adjacent = any(neighbor == key for neighbor, _cost in graph.get(previous, ()) if neighbor in allowed)
                if not adjacent:
                    if len(run) >= 2:
                        candidates.append((bucket, list(run)))
                    run = []
            run.append(key)
        if len(run) >= 2:
            candidates.append((bucket, list(run)))
    center = 0.5 * (min(buckets) + max(buckets)) if buckets else 0.0
    candidates.sort(key=lambda item: (abs(item[0] - center), item[0]))
    return candidates


def _xy_point_segment_distance(point, left, right):
    dx, dy = right[0] - left[0], right[1] - left[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(point[0] - left[0], point[1] - left[1])
    factor = max(0.0, min(1.0, ((point[0] - left[0]) * dx + (point[1] - left[1]) * dy) / length_sq))
    return math.hypot(point[0] - (left[0] + factor * dx), point[1] - (left[1] + factor * dy))


def _covered_keys(keys, cells, fragments, radius):
    covered = set()
    segments = [
        (left, right)
        for fragment in fragments
        for left, right in zip(fragment.points, fragment.points[1:])
    ]
    for key in keys:
        point = cells[key].point
        if any(_xy_point_segment_distance(point, left, right) <= radius + 1e-9 for left, right in segments):
            covered.add(key)
    return covered


def _room_coverage_fragments(space_regions, cells, graph, existing, config, start_index):
    """Add parallel room lanes until the reachable-mask coverage target is met."""
    result = []
    radius = max(config.grid_spacing * 0.75, config.coverage_radius_m * config.units_per_meter)
    for space in space_regions:
        if space.kind != "ROOM":
            continue
        candidates = _space_lane_candidates(space, cells, graph, config)
        if not candidates:
            continue
        relevant = [
            fragment for fragment in existing + result
            if space.parent_floor_id in fragment.region_ids
        ]
        selected_buckets = set()
        minimum_lanes = min(len(candidates), max(1, int(config.room_minimum_lanes)))
        while candidates:
            covered = _covered_keys(space.cell_keys, cells, relevant, radius)
            ratio = len(covered) / max(1, len(space.cell_keys))
            if len(selected_buckets) >= minimum_lanes and ratio >= config.coverage_target_ratio:
                break
            best = None
            for position, (bucket, lane_keys) in enumerate(candidates):
                points = [cells[key].point for key in lane_keys]
                probe = PathFragment("probe", points, (space.parent_floor_id,), "ROOM_COVERAGE")
                gain = len(_covered_keys(space.cell_keys, cells, relevant + [probe], radius) - covered)
                score = (gain, len(points), -position)
                if best is None or score > best[0]:
                    best = (score, position, bucket, points)
            if best is None or (best[0][0] <= 0 and len(selected_buckets) >= minimum_lanes):
                break
            _score, position, bucket, points = best
            candidates.pop(position)
            fragment = PathFragment(
                f"fragment_{start_index + len(result):04d}", points,
                (space.parent_floor_id,), "ROOM_COVERAGE",
            )
            result.append(fragment)
            relevant.append(fragment)
            selected_buckets.add(bucket)
    return result


def generate_fragments(cells, graph, regions, connectors, membership, config,
                       space_regions=None, portals=None):
    fragments = []
    connector_anchors = {}
    for connector in connectors:
        for region_position, region_id in enumerate(connector.region_ids):
            region_keys = set(next(region.cell_keys for region in regions if region.region_id == region_id))
            endpoint = connector.points[0] if region_position == 0 else connector.points[-1]
            candidates = set(key for key in connector.cell_keys if key in region_keys)
            candidates.update(
                neighbor for key in connector.cell_keys for neighbor, _cost in graph[key]
                if neighbor in region_keys
            )
            if candidates:
                anchor = min(candidates, key=lambda key: (_distance(cells[key].point, endpoint), key))
                connector_anchors.setdefault(region_id, set()).add(anchor)
    for region in regions:
        region_graph = _weighted_region_graph(cells, graph, region.cell_keys)
        anchors = connector_anchors.get(region.region_id, set())
        route = _diameter_keys(region_graph, region.cell_keys, preferred=anchors)
        if len(route) >= 2:
            fragments.append(PathFragment(
                f"fragment_{len(fragments):04d}",
                [cells[key].point for key in route],
                (region.region_id,),
                "CENTERLINE",
            ))
        route_set = set(route)
        for anchor in sorted(anchors):
            if anchor in route_set or not route:
                continue
            distances, previous = _dijkstra(region_graph, anchor, region.cell_keys)
            join = min(route, key=lambda key: (distances.get(key, float("inf")), key))
            branch = _path(previous, anchor, join)
            if len(branch) >= 2:
                fragments.append(PathFragment(
                    f"fragment_{len(fragments):04d}",
                    [cells[key].point for key in branch],
                    (region.region_id,),
                    "CONNECTOR_APPROACH",
                ))
    fragments.extend(_room_coverage_fragments(
        list(space_regions or ()), cells, graph, fragments, config, len(fragments)
    ))
    for portal in portals or ():
        if len(portal.points) >= 2:
            fragments.append(PathFragment(
                f"fragment_{len(fragments):04d}", list(portal.points),
                (portal.floor_region_id,), "PORTAL", portal.portal_id,
            ))
    for connector in connectors:
        if len(connector.points) >= 2:
            fragments.append(PathFragment(
                f"fragment_{len(fragments):04d}",
                list(connector.points),
                connector.region_ids,
                connector.kind,
                connector.connector_id,
            ))
    return fragments


def _oriented(fragment, reverse):
    return list(reversed(fragment.points)) if reverse else list(fragment.points)


def _compatible(left, right):
    shared = set(left.region_ids) & set(right.region_ids)
    return bool(shared and (left.connector_id or right.connector_id
                            or left.kind in {"STAIR", "RAMP", "PORTAL", "CONNECTOR_APPROACH"}
                            or right.kind in {"STAIR", "RAMP", "PORTAL", "CONNECTOR_APPROACH"}
                            or set(left.region_ids) == set(right.region_ids)))


def stitch_fragments(fragments, config, segment_validator=None):
    validator = segment_validator or (lambda _a, _b: True)
    work = [PathFragment(item.fragment_id, list(item.points), tuple(item.region_ids), item.kind, item.connector_id)
            for item in fragments if len(item.points) >= 2]
    maximum_distance = config.stitch_distance_m * config.units_per_meter
    maximum_angle = math.radians(config.stitch_angle_degrees)
    while True:
        best = None
        for left_index in range(len(work)):
            for right_index in range(left_index + 1, len(work)):
                left, right = work[left_index], work[right_index]
                if not _compatible(left, right):
                    continue
                for reverse_left in (False, True):
                    lp = _oriented(left, reverse_left)
                    for reverse_right in (False, True):
                        rp = _oriented(right, reverse_right)
                        gap = _distance(lp[-1], rp[0])
                        if gap > maximum_distance or not validator(lp[-1], rp[0]):
                            continue
                        turn = _angle(_sub(lp[-1], lp[-2]), _sub(rp[1], rp[0]))
                        topological_join = gap <= 1e-6 and bool(left.connector_id or right.connector_id)
                        if turn > maximum_angle and not topological_join:
                            continue
                        score = gap + maximum_distance * turn / max(maximum_angle, 1e-9)
                        candidate = (score, left_index, right_index, reverse_left, reverse_right)
                        if best is None or candidate < best:
                            best = candidate
        if best is None:
            break
        _score, left_index, right_index, reverse_left, reverse_right = best
        left, right = work[left_index], work[right_index]
        lp, rp = _oriented(left, reverse_left), _oriented(right, reverse_right)
        points = lp + (rp[1:] if _distance(lp[-1], rp[0]) <= 1e-6 else rp)
        merged = PathFragment(
            left.fragment_id,
            points,
            tuple(sorted(set(left.region_ids) | set(right.region_ids))),
            "STITCHED",
            left.connector_id or right.connector_id,
        )
        work[left_index] = merged
        del work[right_index]
    return work


def _chaikin(points):
    if len(points) < 3:
        return list(points)
    result = [points[0]]
    for left, right in zip(points, points[1:]):
        result.extend((_lerp(left, right, 0.25), _lerp(left, right, 0.75)))
    result.append(points[-1])
    return result


def smooth_fragment(fragment, config, segment_validator=None):
    validator = segment_validator or (lambda _a, _b: True)
    points = list(fragment.points)
    for _iteration in range(max(0, config.smoothing_iterations)):
        candidate = _chaikin(points)
        if all(validator(left, right) for left, right in zip(candidate, candidate[1:])):
            points = candidate
        else:
            break
    return PathFragment(fragment.fragment_id, points, fragment.region_ids, fragment.kind, fragment.connector_id)


def plan_walkable_paths(
    cell_values,
    config: PlannerConfig,
    segment_validator=None,
    *,
    edge_validator=None,
    reachable_seed_key=None,
    stitch_fragments_enabled=True,
):
    cells, graph = build_walkable_graph(cell_values, config, edge_validator=edge_validator)
    detected_cell_count = len(cells)
    if reachable_seed_key is not None:
        reachable = reachable_cell_keys(graph, reachable_seed_key)
        cells, graph = _restrict_graph(cells, graph, reachable)
    regions, membership = identify_floor_regions(cells, graph, config)
    connectors = identify_connectors(cells, graph, regions, membership, config)
    if reachable_seed_key is not None:
        reachable_regions = _reachable_region_ids(
            cells, graph, regions, membership, connectors, tuple(reachable_seed_key)
        )
        regions = [region for region in regions if region.region_id in reachable_regions]
        connectors = [
            connector for connector in connectors
            if set(connector.region_ids) <= reachable_regions
        ]
        allowed_keys = {
            key for region in regions for key in region.cell_keys
        } | {
            key for connector in connectors for key in connector.cell_keys
        }
        cells, graph = _restrict_graph(cells, graph, allowed_keys)
        membership = {
            key: region_id for key, region_id in membership.items()
            if region_id in reachable_regions and key in cells
        }
    cells, maximum_mask_distance = _apply_distance_field(cells, graph, config)
    space_regions, portals = classify_reachable_space(cells, graph, regions, config)
    mask_validator = _walkable_mask_validator(cells, config)
    external_validator = segment_validator or (lambda _left, _right: True)

    def final_validator(left, right):
        return mask_validator(left, right) and external_validator(left, right)

    raw = generate_fragments(
        cells, graph, regions, connectors, membership, config,
        space_regions=space_regions, portals=portals,
    )
    stitched = (
        stitch_fragments(raw, config, segment_validator=final_validator)
        if stitch_fragments_enabled else list(raw)
    )
    final = [smooth_fragment(fragment, config, segment_validator=final_validator) for fragment in stitched]
    coverage_radius = max(
        config.grid_spacing * 0.75,
        config.coverage_radius_m * config.units_per_meter,
    )
    covered = _covered_keys(cells, cells, final, coverage_radius)
    room_regions = [space for space in space_regions if space.kind == "ROOM"]
    uncovered_rooms = [
        space.region_id for space in room_regions
        if not (_covered_keys(space.cell_keys, cells, final, coverage_radius))
    ]
    stats = {
        "walkable_cell_count": len(cells),
        "detected_walkable_cell_count": detected_cell_count,
        "excluded_unreachable_cell_count": detected_cell_count - len(cells),
        "reachable_seed_key": list(reachable_seed_key) if reachable_seed_key is not None else None,
        "distance_field_max_m": maximum_mask_distance / max(config.units_per_meter, 1e-9),
        "floor_region_count": len(regions),
        "connector_count": len(connectors),
        "stair_connector_count": sum(item.kind == "STAIR" for item in connectors),
        "space_region_count": len(space_regions),
        "room_region_count": len(room_regions),
        "corridor_region_count": sum(item.kind == "CORRIDOR" for item in space_regions),
        "portal_count": len(portals),
        "path_spatial_coverage_ratio": len(covered) / max(1, len(cells)),
        "uncovered_room_count": len(uncovered_rooms),
        "uncovered_room_ids": uncovered_rooms,
        "raw_fragment_count": len(raw),
        "final_spline_count": len(final),
    }
    return PlanResult(
        regions=regions, connectors=connectors, raw_fragments=raw,
        final_fragments=final, graph=graph, stats=stats,
        space_regions=space_regions, portals=portals,
    )


def _metrics(points):
    cumulative = [0.0]
    for left, right in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + _distance(left, right))
    return cumulative


def _at(points, cumulative, distance):
    distance = max(0.0, min(distance, cumulative[-1]))
    index = 1
    while index < len(cumulative) - 1 and cumulative[index] < distance:
        index += 1
    span = max(1e-12, cumulative[index] - cumulative[index - 1])
    factor = (distance - cumulative[index - 1]) / span
    return _lerp(points[index - 1], points[index], factor), _normalize(_sub(points[index], points[index - 1]))


def sample_arc_length(points, base_spacing, *, curvature_gain=0.75, connector_gain=0.45,
                      heading_threshold_degrees=18.0):
    """Sample by true polyline arc length, densifying turns and vertical travel."""
    points = [_point(point) for point in points]
    if len(points) < 2:
        return [ArcSample(0.0, points[0], (1.0, 0.0, 0.0), 0.0, True, False)] if points else []
    cumulative = _metrics(points)
    total = cumulative[-1]
    if total <= 1e-12:
        return [ArcSample(0.0, points[0], (1.0, 0.0, 0.0), 0.0, True, False)]
    threshold = math.radians(heading_threshold_degrees)
    critical = {0.0, total}
    for index in range(1, len(points) - 1):
        if _angle(_sub(points[index], points[index - 1]), _sub(points[index + 1], points[index])) >= threshold:
            critical.add(cumulative[index])
    distances = set(critical)
    cursor = 0.0
    while cursor < total - 1e-9:
        point, tangent = _at(points, cumulative, cursor)
        probe = min(total, cursor + max(base_spacing * 0.35, 1e-6))
        _next_point, next_tangent = _at(points, cumulative, probe)
        curvature = _angle(tangent, next_tangent) / max(probe - cursor, 1e-9)
        grade = abs(tangent[2])
        spacing = base_spacing / (1.0 + curvature_gain * curvature * base_spacing + connector_gain * grade)
        spacing = max(base_spacing * 0.35, min(base_spacing, spacing))
        cursor = min(total, cursor + spacing)
        distances.add(cursor)
    result = []
    for distance in sorted(distances):
        point, tangent = _at(points, cumulative, distance)
        probe = min(total, distance + max(base_spacing * 0.25, 1e-6))
        _probe_point, next_tangent = _at(points, cumulative, probe)
        curvature = _angle(tangent, next_tangent) / max(probe - distance, 1e-9) if probe > distance else 0.0
        result.append(ArcSample(
            distance, point, tangent, curvature,
            any(abs(distance - item) <= 1e-7 for item in critical),
            abs(tangent[2]) >= 0.08,
        ))
    return result
