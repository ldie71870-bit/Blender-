"""Blender adapter for layered contour routes. Coordinates in metres internally."""
from __future__ import annotations

import json
import math
import time
from collections import defaultdict

import bpy
from mathutils import Vector
from . import contour_planner as planner
from . import path_planner_3d as topology
from . import detail_coverage


class SceneProbe:
    def __init__(self, scene, scale, margin, step):
        self.scene = scene
        self.depsgraph = bpy.context.evaluated_depsgraph_get()
        self.scale, self.margin, self.step = scale, margin, step
        self.point_cache, self.segment_cache = {}, {}
        self.rays = 0
        self.directions = [Vector(v).normalized() for v in
                           [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)] +
                           [(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)]]

    def cast(self, point, direction, distance):
        self.rays += 1
        hit, position, normal, index, obj, matrix = self.scene.ray_cast(
            self.depsgraph, Vector(point)/self.scale, Vector(direction), distance=distance/self.scale)
        if not hit:
            return None
        return tuple(position*self.scale), normal, obj

    def point_clear(self, point):
        key = tuple(round(v, 5) for v in point)
        if key in self.point_cache:
            return self.point_cache[key]
        down = self.cast(point, (0,0,-1), self.vertical_range)
        up = self.cast(point, (0,0,1), self.vertical_range)
        # Reject exterior and points inside consistently oriented closed solids.
        valid = bool(down and up and down[1].z >= -.05 and up[1].z <= .05)
        if valid:
            for direction in self.directions:
                if self.cast(point, direction, self.margin):
                    valid = False
                    break
        if len(self.point_cache) >= 200000:
            self.point_cache.clear()
        self.point_cache[key] = valid
        return valid

    def segment_clear(self, left, right):
        a, b = tuple(left), tuple(right)
        key = tuple(sorted((tuple(round(v,5) for v in a), tuple(round(v,5) for v in b))))
        if key in self.segment_cache:
            return self.segment_cache[key]
        valid = self.point_clear(a) and self.point_clear(b)
        delta = Vector(b)-Vector(a)
        distance = delta.length
        if valid and distance > 1e-7:
            direction = delta/distance
            axis = Vector((0,0,1)) if abs(direction.z) < .9 else Vector((1,0,0))
            side = direction.cross(axis).normalized()
            up = direction.cross(side).normalized()
            offsets = [Vector((0,0,0))]+[
                self.margin*(math.cos(i*math.tau/8)*side+math.sin(i*math.tau/8)*up)
                for i in range(8)]
            for offset in offsets:
                if self.cast(Vector(a)+offset, direction, distance):
                    valid = False
                    break
            if valid:
                count = max(1, math.ceil(distance/max(.05, min(.15, self.margin*.75))))
                for index in range(1, count):
                    if not self.point_clear(Vector(a)+delta*(index/count)):
                        valid = False
                        break
        if len(self.segment_cache) >= 100000:
            self.segment_cache.clear()
        self.segment_cache[key] = valid
        return valid


def geometry_bounds(scene, probe):
    low, high = Vector((math.inf,)*3), Vector((-math.inf,)*3)
    found = False
    for instance in probe.depsgraph.object_instances:
        obj = instance.object
        if obj.type != 'MESH' or obj.get('gs_camera_mesh_visual'):
            continue
        if not instance.is_instance and not obj.original.visible_get():
            continue
        for corner in obj.bound_box:
            point = (instance.matrix_world @ Vector(corner))*probe.scale
            for i in range(3):
                low[i], high[i] = min(low[i], point[i]), max(high[i], point[i])
        found = True
    if not found:
        raise ValueError('没有可见网格可用于排线')
    return low, high


def sample_supports(probe, bounds, step, floor_collection, progress):
    low, high = bounds
    origin = (low.x, low.y)
    nx, ny = math.ceil((high.x-low.x)/step)+1, math.ceil((high.y-low.y)/step)+1
    if nx*ny > 100000:
        raise ValueError(f'采样范围过大（{nx*ny:,} 列）；请隐藏无关模型或增大采样间距')
    allowed = {obj.original.as_pointer() for obj in floor_collection.all_objects} if floor_collection else None
    cells, truncated = {}, 0
    depth = high.z-low.z+2
    for iy in range(ny):
        y = origin[1]+iy*step
        for ix in range(nx):
            x = origin[0]+ix*step
            z = high.z+.5
            last_floor = math.inf
            for hit_index in range(128):
                hit = probe.cast((x,y,z), (0,0,-1), depth)
                if hit is None or hit[0][2] < low.z-.1:
                    break
                position, normal, obj = hit
                floor = position[2]
                if normal.z >= .55 and last_floor-floor > .04 and (
                        allowed is None or obj.original.as_pointer() in allowed):
                    key = (ix, iy, round(floor/.01))
                    cells[key] = (x,y,floor)
                    last_floor = floor
                z = floor-.0005
            else:
                truncated += 1
        if iy % 4 == 0:
            progress('识别地面和夹层', iy+1, ny)
    if truncated:
        raise ValueError(f'{truncated} 列几何交点过多，未完成采样；请用简化建筑外壳重试')
    return cells, origin, nx*ny


def make_layer_graph(supports, height, probe, step, max_step, progress):
    values = []
    for index, (key, floor) in enumerate(supports.items()):
        point = (floor[0], floor[1], floor[2]+height)
        if probe.point_clear(point):
            values.append(topology.WalkableCell(key, point, floor[2], probe.margin))
        if index % 400 == 0:
            progress('检查本层障碍', index+1, len(supports))
    by_xy = defaultdict(list)
    for key, floor in supports.items():
        by_xy[key[:2]].append(floor[2])
    first = next(iter(supports.items()), None)
    if not first:
        return {}, {}, probe.segment_clear
    key, floor = first
    origin = (floor[0]-key[0]*step, floor[1]-key[1]*step)
    def supported_segment(a, b):
        count = max(1, math.ceil(math.dist(a,b)/(step*.45)))
        for i in range(count+1):
            t = i/count
            x,y,z = (a[j]+t*(b[j]-a[j]) for j in range(3))
            grid = (round((x-origin[0])/step), round((y-origin[1])/step))
            floors = by_xy.get(grid, ())
            if not floors or min(abs(z-height-f) for f in floors) > max_step+.02:
                return False
        return probe.segment_clear(a,b)
    config = topology.PlannerConfig(grid_spacing=step, maximum_connector_step_m=max_step)
    progress('连接可通行区域', 0, 1)
    cells, graph = topology.build_walkable_graph(values, config, edge_validator=supported_segment)
    return cells, graph, supported_segment


def choose_components(cells, graph, supports, seed, mode, height, min_area, step):
    components = [c for c in topology._components(graph) if len(c)*step*step >= min_area]
    if not components:
        return {}, {}, {'component_count': 0, 'seed_distance': None}
    if mode != 'ALL':
        # Anchor to the floor underneath the seed, independently of camera layer.
        lower = [(math.hypot(p[0]-seed[0],p[1]-seed[1]), seed[2]-p[2], key)
                 for key,p in supports.items() if p[2] <= seed[2]+.15]
        if not lower:
            raise ValueError('起点下方没有识别到地面；请将 3D 游标放在目标房间内部')
        nearby = [item for item in lower if item[0] <= step*1.5]
        floor_key = min(nearby, key=lambda item: (item[1], item[0]))[2] if nearby else min(lower)[2]
        floor = supports[floor_key]
        expected = (seed[0],seed[1],floor[2]+height)
        candidates = [(math.dist(cells[key].point,expected), n, key)
                      for n,c in enumerate(components) for key in c
                      if abs(cells[key].floor_z-floor[2]) < .25]
        if not candidates:
            return {}, {}, {'component_count': 0, 'seed_distance': None}
        distance, n, _key = min(candidates)
        if distance > max(2.0,step*4):
            raise ValueError('起点距离本层可通行区域超过 2 米；请移动起点或检查层高')
        components = [components[n]]
    else:
        distance = None
    keep = {key for component in components for key in component}
    return ({key:cells[key] for key in keep},
            {key:[(n,cost) for n,cost in graph[key] if n in keep] for key in keep},
            {'component_count':len(components),'seed_distance':distance})


def generate(scene, settings, progress=None):
    start = time.monotonic()
    progress = progress or (lambda stage, current, total: None)
    scale = max(1e-9, scene.unit_settings.scale_length)
    step = float(settings.contour_probe_spacing)
    margin = float(settings.contour_clearance)
    lane_gap = float(settings.floorplan_spacing)
    if step > lane_gap*.6:
        raise ValueError('采样间距应小于排线间距的 60%；请提高采样精度')
    probe = SceneProbe(scene, scale, margin, step)
    bounds = geometry_bounds(scene, probe)
    probe.vertical_range = bounds[1].z-bounds[0].z+2
    supports, origin, columns = sample_supports(probe, bounds, step, settings.contour_floor_collection, progress)
    raw_support_count = len(supports)
    if not settings.contour_floor_collection:
        supports = planner.structural_supports(supports, step, settings.contour_min_floor_area, settings.contour_max_step)
    if not supports:
        raise ValueError('没有识别到向上的地面；请检查地面集合和模型法线')
    mode = settings.floorplan_layer_mode
    specs = [('Low',float(settings.floorplan_low_height))]
    if mode in {'THREE','FOUR'}:
        specs.append(('Middle',float(settings.floorplan_mid_height)))
    if mode == 'FOUR':
        specs.append(('Middle_2',float(settings.floorplan_high_height)))
    specs.append(('High',float(settings.floorplan_top_height)))
    if mode == 'ONE':
        specs = [('Middle', float(settings.floorplan_mid_height))]
    # Duplicate height settings produce one layer, with contiguous layer metadata.
    specs = list(dict((round(height,5),(name,height)) for name,height in specs).values())
    seed_mode = settings.floorplan_seed_mode
    seed_object = settings.floorplan_seed_object if seed_mode == 'OBJECT' else scene.camera if seed_mode == 'CAMERA' else None
    seed = tuple((seed_object.matrix_world.translation if seed_object else scene.cursor.location)*scale)
    all_routes, layer_stats, layer_models = [], [], []
    for layer_index,(label,height) in enumerate(specs):
        probe.point_cache.clear()
        probe.segment_cache.clear()
        progress(f'{label} 层',layer_index,len(specs))
        cells, graph, layer_valid = make_layer_graph(supports,height,probe,step,settings.contour_max_step,progress)
        cells, graph, selection = choose_components(cells,graph,supports,seed,settings.floorplan_space_mode,
                                                     height,settings.contour_min_area,step)
        if not cells:
            layer_stats.append(dict(label=label,height=height,route_count=0,free_cells=0,
                                    note='该层没有符合净空和起点条件的区域'))
            continue
        progress('提取轮廓与连接主线', 0, 1)
        routes, stats = planner.plan_layer(cells,graph,origin,step,lane_gap,layer_valid,
                                           minimum_length=.8,maximum_bridge=settings.contour_max_bridge,
                                           smoothing=settings.contour_smoothing)
        stats.update(selection,label=label,height=height)
        layer_stats.append(stats)
        layer_models.append(dict(index=layer_index,label=label,cells=cells,graph=graph,valid=layer_valid))
        all_routes.extend((layer_index,label,route) for route in routes)
    if not all_routes:
        raise ValueError('未生成安全路径；检查起点、层高、净空距离，或指定地面集合')
    main_route_count = len(all_routes)
    detail_report = None
    if settings.contour_detail_enabled:
        detail_routes, detail_report = detail_coverage.refine(all_routes,layer_models,probe,settings,progress)
        all_routes.extend(detail_routes)
    report = dict(version='1.6.1',method='LAYERED_CONTOUR',columns=columns,support_samples=len(supports),
                  raw_support_samples=raw_support_count, distinct_z_bands=len({round(p[2]/.1) for p in supports.values()}),
                  probe_spacing_m=step,clearance_m=margin,lane_spacing_m=lane_gap,layers=layer_stats,
                  route_count=len(all_routes),ray_count=probe.rays,elapsed_seconds=time.monotonic()-start,
                  collision_check='14-direction clearance, 9-ray segment tube, <=0.15m interior sampling',
                  main_route_count=main_route_count, detail_coverage=detail_report,
                  surface_visibility_evaluated=detail_report is not None,
                  floor_source=settings.contour_floor_collection.name if settings.contour_floor_collection else 'AUTO')
    # Commit only after every layer validates. Never clear existing/user collections.
    collection = bpy.data.collections.new('GS_Contour_Preview')
    objects, created_data = [], []
    try:
        scene.collection.children.link(collection)
        colors = [(0.15,.55,.9,1),(.15,.8,.45,1),(.85,.5,.15,1),(.75,.3,.8,1)]
        for index,(layer_index,label,route) in enumerate(all_routes):
            name = f'GS_{"Detail" if route.kind == "DETAIL" else "Contour"}_{label}_{index+1:03d}'
            curve = bpy.data.curves.new(name,'CURVE')
            created_data.append(curve)
            curve.dimensions = '3D'
            spline = curve.splines.new('POLY')
            spline.points.add(len(route.points)-1)
            for point,coordinate in zip(spline.points,route.points):
                point.co = tuple(v/scale for v in coordinate)+(1.0,)
            obj = bpy.data.objects.new(name,curve)
            objects.append(obj)
            collection.objects.link(obj)
            obj.color = (1.0,.65,.1,1) if route.kind == 'DETAIL' else colors[layer_index % len(colors)]
            obj.show_in_front = True
            obj['gs_contour_version'] = '1.6.1'
            obj['gs_floorplan_layer'] = label
            obj['gs_explicit_capture_height'] = True
            obj['gs_capture_layer_index'] = 1 if len(specs) == 1 else layer_index
            obj['gs_capture_layer_count'] = 3 if len(specs) == 1 else len(specs)
            obj['gs_route_kind'] = route.kind
            obj['gs_route_role'] = 'DETAIL' if route.kind == 'DETAIL' else 'MAIN'
            if route.kind == 'DETAIL':
                obj['gs_detail_gain'] = route.detail_gain
                obj['gs_detail_target_objects'] = json.dumps(route.target_objects,ensure_ascii=False)
                obj['gs_detail_min_samples'] = 5
        collection['gs_contour_report'] = json.dumps(report,ensure_ascii=False)
    except Exception:
        for obj in objects:
            bpy.data.objects.remove(obj,do_unlink=True)
        for curve in created_data:
            if curve.users == 0:
                bpy.data.curves.remove(curve)
        bpy.data.collections.remove(collection)
        raise
    scene['gs_contour_last_report'] = json.dumps(report,ensure_ascii=False)
    return dict(primary=objects[0],collection=collection,objects=objects,
                points=sum(len(route.points) for _,_,route in all_routes),report=report)
