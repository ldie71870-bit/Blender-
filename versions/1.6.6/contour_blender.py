"""Blender adapter for layered contour routes. Coordinates in metres internally."""
from __future__ import annotations

import json
import math
import time
from bisect import bisect_right
from collections import defaultdict

import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from . import contour_planner as planner
from . import path_planner_3d as topology
from . import detail_coverage
from . import layer_access


class SceneProbe:
    def __init__(self, scene, scale, margin, step, geometry=None):
        self.scene = scene
        self.depsgraph = bpy.context.evaluated_depsgraph_get()
        self.scale, self.margin, self.step = scale, margin, step
        self.point_cache, self.segment_cache = {}, {}
        self.rays = 0
        self.directions = [Vector(v).normalized() for v in
                           [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)] +
                           [(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)]]
        self.geometry=geometry if geometry is not None else self.build_geometry()

    def build_geometry(self):
        vertices,polygons,ends,owners=[],[],[],[]
        for instance in self.depsgraph.object_instances:
            obj=instance.object
            if obj.type!='MESH' or obj.get('gs_camera_mesh_visual'):continue
            if not instance.is_instance and not obj.original.visible_get():continue
            mesh=obj.to_mesh()
            if mesh is None:continue
            try:
                matrix=instance.matrix_world.copy();offset=len(vertices)
                vertices.extend((matrix@v.co)*self.scale for v in mesh.vertices)
                mirrored=matrix.to_3x3().determinant()<0
                for polygon in mesh.polygons:
                    indices=tuple(offset+i for i in polygon.vertices)
                    if len(indices)>=3:polygons.append(tuple(reversed(indices)) if mirrored else indices)
                ends.append(len(polygons));owners.append(obj.original)
            finally:obj.to_mesh_clear()
        return (BVHTree.FromPolygons(vertices,polygons,all_triangles=False) if polygons else None,ends,owners)

    def cast(self, point, direction, distance):
        # Use Blender's native hit ownership and normals for surface semantics.
        # Coincident opposite-facing faces can tie differently in a merged BVH.
        start=Vector(point);direction=Vector(direction).normalized();remaining=distance
        for _ in range(64):
            self.rays+=1
            hit,position,normal,index,obj,_=self.scene.ray_cast(self.depsgraph,start/self.scale,direction,distance=remaining/self.scale)
            if not hit:return None
            position*=self.scale
            if not obj.get('gs_contour_version'):return tuple(position),normal,obj
            remaining-=(position-start).length+.025
            if remaining<=0:return None
            start=position+direction*.025
        return tuple(position),normal,obj

    def blocked(self, point, direction, distance):
        self.rays+=1
        bvh,ends,owners=self.geometry
        if bvh is None:return False
        position,normal,index,_=bvh.ray_cast(Vector(point),Vector(direction).normalized(),distance)
        return position is not None

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
                if self.blocked(point, direction, self.margin):
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
                if self.blocked(Vector(a)+offset, direction, distance):
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


def make_layer_graph(supports, height, probe, step, max_step, progress, ceiling_gap=None, minimum_height=.5):
    values = []
    expected_by_xy=defaultdict(list)
    for index, (key, floor) in enumerate(supports.items()):
        target_z=floor[2]+height
        if ceiling_gap is not None and not probe.point_clear((floor[0],floor[1],target_z)):
            # Start above the middle layer, so a table underside below the
            # camera cannot be mistaken for the room ceiling.
            overhead=probe.cast((floor[0],floor[1],floor[2]+minimum_height+probe.margin+.02),(0,0,1),probe.vertical_range)
            if not overhead or overhead[1].z>.05:continue
            target_z=min(target_z,overhead[0][2]-max(ceiling_gap,probe.margin+.03))
            if target_z-floor[2]<minimum_height:continue
        expected_by_xy[key[:2]].append(target_z)
        point = (floor[0], floor[1], target_z)
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
            expected=expected_by_xy.get(grid, ())
            if not expected or min(abs(z-target) for target in expected) > max_step+.02:
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


def build_models(supports, specs, probe, step, settings, seed, progress):
    models=[]
    for index,(label,height) in enumerate(specs):
        progress(label+' 层通行检查',index,len(specs))
        ceiling_gap=settings.floorplan_ceiling_offset if label=='High' and settings.contour_adapt_top else None
        minimum_height=(settings.floorplan_mid_height+.15 if len(specs)>=3 else settings.floorplan_low_height+.15)
        cells,graph,valid=make_layer_graph(supports,height,probe,step,settings.contour_max_step,progress,ceiling_gap,minimum_height)
        models.append(dict(index=index,label=label,height=height,cells=cells,graph=graph,valid=valid,step=step))
    if settings.floorplan_space_mode=='ALL':
        keep=[{k for component in topology._components(m['graph']) if len(component)*step*step>=settings.contour_min_area for k in component} for m in models]
    else:
        seed_index=None
        for i in sorted(range(len(models)),key=lambda i:abs(i-(len(models)-1)/2)):
            m=models[i]
            try:
                selected,_,_=choose_components(m['cells'],m['graph'],supports,seed,'REACHABLE',m['height'],settings.contour_min_area,step)
            except ValueError:continue
            if selected:
                seed_index=i;break
        if seed_index is None:raise ValueError('起点附近没有安全的可达位置；请将起点放入目标室内空间')
        progress('检查跨高度通行，保留家具缝隙',0,1)
        keep=layer_access.reachable_models(models,seed_index,selected,probe.segment_clear)
    for m,keys in zip(models,keep):
        m['cells']={k:v for k,v in m['cells'].items() if k in keys}
        m['graph']={k:[(n,c) for n,c in m['graph'][k] if n in keys] for k in keys}
        m['component_count']=len(topology._components(m['graph']))
        m['adapted_top_cells']=sum(c.point[2]-c.floor_z<m['height']-.01 for c in m['cells'].values())
    return models


def show_latest_preview(scene, collection):
    current={obj.as_pointer() for obj in collection.all_objects}
    for obj in scene.objects:
        if obj.type=='CURVE' and obj.get('gs_contour_version'):
            obj.hide_set(obj.as_pointer() not in current)


def generate(scene, settings, progress=None):
    # Renderable preview curves must not become ceilings or obstacles on rerun.
    previous=[(obj,obj.hide_get()) for obj in scene.objects if obj.type=='CURVE' and obj.get('gs_contour_version')]
    result=None
    try:
        for obj,_ in previous:obj.hide_set(True)
        bpy.context.view_layer.update()
        result=_generate(scene,settings,progress)
    finally:
        for obj,hidden in previous:obj.hide_set(hidden)
        bpy.context.view_layer.update()
    show_latest_preview(scene,result['collection'])
    return result


def _generate(scene, settings, progress=None):
    start = time.monotonic()
    progress = progress or (lambda stage, current, total: None)
    scale = max(1e-9, scene.unit_settings.scale_length)
    step = float(settings.contour_probe_spacing)
    margin = float(settings.contour_clearance)
    lane_gap = float(settings.floorplan_spacing)
    if step > lane_gap*.6:
        raise ValueError('采样间距应小于排线间距的 60%；请提高采样精度')
    progress('建立场景几何查询',0,1)
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
    all_routes, layer_stats = [], []
    layer_models=build_models(supports,specs,probe,step,settings,seed,progress)
    for model in layer_models:
        layer_index,label,height=model['index'],model['label'],model['height']
        cells,graph,layer_valid=model['cells'],model['graph'],model['valid']
        if not cells:
            layer_stats.append(dict(label=label,height=height,route_count=0,free_cells=0,
                                    note='该层没有符合净空和起点条件的区域'))
            continue
        progress('提取轮廓与连接主线', 0, 1)
        routes, stats = planner.plan_layer(cells,graph,origin,step,lane_gap,layer_valid,
                                           minimum_length=.8,maximum_bridge=settings.contour_max_bridge,
                                           smoothing=settings.contour_smoothing)
        stats.update(component_count=model['component_count'],adapted_top_cells=model['adapted_top_cells'],label=label,height=height)
        layer_stats.append(stats)
        all_routes.extend((layer_index,label,route) for route in routes)
    if not all_routes:
        raise ValueError('未生成安全路径；检查起点、层高、净空距离，或指定地面集合')
    main_route_count = len(all_routes)
    detail_report = None
    if settings.contour_detail_enabled:
        detail_step=min(step,float(settings.contour_detail_grid))
        detail_margin=min(margin,float(settings.contour_detail_clearance))
        detail_probe=SceneProbe(scene,scale,detail_margin,detail_step,geometry=probe.geometry)
        detail_probe.vertical_range=probe.vertical_range
        if abs(detail_step-step)<1e-6:
            detail_supports=supports
        else:
            detail_supports,_,_=sample_supports(detail_probe,bounds,detail_step,settings.contour_floor_collection,progress)
            if not settings.contour_floor_collection:
                detail_supports=planner.structural_supports(detail_supports,detail_step,settings.contour_min_floor_area,settings.contour_max_step)
        detail_models=build_models(detail_supports,specs,detail_probe,detail_step,settings,seed,progress)
        detail_routes, detail_report = detail_coverage.refine(all_routes,detail_models,detail_probe,settings,progress)
        detail_report['clearance_m']=detail_margin
        detail_report['grid_spacing_m']=detail_step
        detail_report['reachable_cells_by_layer']={m['label']:len(m['cells']) for m in detail_models}
        probe.rays+=detail_probe.rays
        all_routes.extend(detail_routes)
    report = dict(version='1.6.4',method='LAYERED_CONTOUR',columns=columns,support_samples=len(supports),
                  raw_support_samples=raw_support_count, distinct_z_bands=len({round(p[2]/.1) for p in supports.values()}),
                  probe_spacing_m=step,clearance_m=margin,lane_spacing_m=lane_gap,layers=layer_stats,
                  route_count=len(all_routes),ray_count=probe.rays,elapsed_seconds=time.monotonic()-start,
                  collision_check='14-direction clearance, 9-ray segment tube, <=0.15m interior sampling',
                  main_route_count=main_route_count, detail_coverage=detail_report,
                  surface_visibility_evaluated=detail_report is not None,
                  floor_source=settings.contour_floor_collection.name if settings.contour_floor_collection else 'AUTO')
    # Commit only after every layer validates. Never clear existing/user collections.
    collection = bpy.data.collections.new('GS_Contour_Preview')
    objects, created_data, created_children, created_materials = [], [], [], []
    try:
        scene.collection.children.link(collection)
        colors = [(0.15,.55,.9,1),(.15,.8,.45,1),(.85,.5,.15,1),(.75,.3,.8,1)]
        groups={}
        materials={}
        labels={'Low':'低层','Middle':'中层','High':'顶层','Middle_2':'高中层'}
        for index,(layer_index,label,route) in enumerate(all_routes):
            role='DETAIL' if route.kind=='DETAIL' else 'MAIN'
            group_key=(layer_index,role)
            color=(.95,.24,.09,1) if role=='DETAIL' else colors[layer_index % len(colors)]
            if group_key not in groups:
                child=bpy.data.collections.new(f'{labels.get(label,label)} · {"细部" if role=="DETAIL" else "主线"}')
                created_children.append(child);collection.children.link(child);groups[group_key]=child
                mat=bpy.data.materials.new(f'GS_Path_{label}_{role}')
                created_materials.append(mat);mat.diffuse_color=color;mat.use_nodes=True
                nodes=mat.node_tree.nodes;nodes.clear()
                emission=nodes.new('ShaderNodeEmission');emission.inputs['Color'].default_value=color
                emission.inputs['Strength'].default_value=.8
                output=nodes.new('ShaderNodeOutputMaterial');mat.node_tree.links.new(emission.outputs[0],output.inputs['Surface'])
                materials[group_key]=mat
            name = f'GS_{"Detail" if route.kind == "DETAIL" else "Contour"}_{label}_{index+1:03d}'
            curve = bpy.data.curves.new(name,'CURVE')
            created_data.append(curve)
            curve.dimensions = '3D'
            curve.bevel_depth=.006/scale
            curve.bevel_resolution=2
            curve.materials.append(materials[group_key])
            spline = curve.splines.new('POLY')
            spline.points.add(len(route.points)-1)
            for point,coordinate in zip(spline.points,route.points):
                point.co = tuple(v/scale for v in coordinate)+(1.0,)
            obj = bpy.data.objects.new(name,curve)
            objects.append(obj)
            groups[group_key].objects.link(obj)
            obj.color = color
            obj.show_in_front = False
            obj.hide_render = True
            obj['gs_contour_version'] = '1.6.4'
            obj['gs_floorplan_layer'] = label
            obj['gs_explicit_capture_height'] = True
            obj['gs_capture_layer_index'] = 1 if len(specs) == 1 else layer_index
            obj['gs_capture_layer_count'] = 3 if len(specs) == 1 else len(specs)
            obj['gs_route_kind'] = route.kind
            obj['gs_route_role'] = 'DETAIL' if route.kind == 'DETAIL' else 'MAIN'
            if route.kind == 'DETAIL':
                obj['gs_detail_gain'] = route.detail_gain
                obj['gs_gap_gain'] = getattr(route,'gap_gain',0)
                obj['gs_detail_target_objects'] = json.dumps(route.target_objects,ensure_ascii=False)
                obj['gs_detail_min_samples'] = 5
        collection['gs_contour_report'] = json.dumps(report,ensure_ascii=False)
    except Exception:
        for obj in objects:
            bpy.data.objects.remove(obj,do_unlink=True)
        for curve in created_data:
            if curve.users == 0:
                bpy.data.curves.remove(curve)
        for child in created_children:bpy.data.collections.remove(child)
        for mat in created_materials:
            if mat.users==0:bpy.data.materials.remove(mat)
        bpy.data.collections.remove(collection)
        raise
    scene['gs_contour_last_report'] = json.dumps(report,ensure_ascii=False)
    return dict(primary=objects[0],collection=collection,objects=objects,
                points=sum(len(route.points) for _,_,route in all_routes),report=report)
