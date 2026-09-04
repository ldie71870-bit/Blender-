"""Scene-ray discovery and targeted short-line refinement of contour routes.

The denominator is explicitly a sampled, candidate-observable surface set, not
all triangles in the scene. Detail lines are kept separate from main routes.
"""
import math
from collections import defaultdict
from mathutils import Vector
from mathutils.kdtree import KDTree
from . import contour_planner as planner
from . import detail_metrics as metrics
from . import layer_access
from . import path_planner_3d as topology


def resample(points, spacing):
    result=[]
    carry=0.
    for a,b in zip(points,points[1:]):
        distance=math.dist(a,b)
        if distance<1e-8:continue
        while carry<=distance:
            result.append(tuple(a[j]+(b[j]-a[j])*carry/distance for j in range(3)))
            carry+=spacing
        carry-=distance
    if points:result.append(tuple(points[-1]))
    return result


def kd(points):
    tree=KDTree(len(points))
    for i,p in enumerate(points):tree.insert(p,i)
    tree.balance()
    return tree


def gap_lines(main_routes,models,probe):
    """Follow long missing pockets once instead of scattering crossing strokes."""
    lines=[]
    for mi,model in enumerate(models):
        cells,graph=model['cells'],model['graph']
        if not cells:continue
        key=next(iter(cells));p=cells[key].point
        origin=(p[0]-key[0]*probe.step,p[1]-key[1]*probe.step)
        main=[route for index,_,route in main_routes if index==model['index']]
        covered=planner.covered_keys(main,cells,graph,max(.28,probe.step*1.5),origin,probe.step)
        missing=set(cells)-covered
        for keys in sorted(topology._components(graph,missing),key=len,reverse=True)[:40]:
            if len(keys)<3:continue
            ordered=topology._diameter_keys(graph,keys)
            points=[cells[k].point for k in ordered]
            if planner.length(points)<.4:continue
            # Limit very large residual regions while retaining a coherent run.
            pieces=[];piece=[points[0]];travel=0.
            for p in points[1:]:
                travel+=math.dist(piece[-1],p);piece.append(p)
                if travel>=4.0:pieces.append(piece);piece=[p];travel=0.
            if len(piece)>1:pieces.append(piece)
            for piece in pieces:
                piece=planner.simplify(piece,probe.step*.4,model['valid'])
                piece=planner.smooth(piece,model['valid'],2)
                if planner.length(piece)>=.4:
                    lines.append(dict(points=piece,length=planner.length(piece),model=mi,
                                      center=piece[len(piece)//2],kind='GAP',
                                      space_targets={(mi,*k) for k in planner.covered_keys([planner.Route(piece)],cells,graph,.20,origin,probe.step)&missing}))
    return lines


def refine(main_routes, models, probe, settings, progress):
    precision=float(settings.contour_detail_precision)
    max_distance=float(settings.contour_detail_distance)
    baseline=float(settings.contour_detail_baseline)
    angle=float(settings.contour_detail_angle)
    sample_points=[]
    samples_by_layer=defaultdict(list)
    seen=set()
    for layer_index,_,route in main_routes:
        for point in resample(route.points,.30):
            key=(layer_index,*(round(v/.10) for v in point))
            if key not in seen:
                seen.add(key);sample_points.append(point);samples_by_layer[layer_index].append(point)
    if not sample_points:return [],dict(target_count=0,detail_lines=0)
    main_trees={i:kd(points) for i,points in samples_by_layer.items()}
    candidates=[]
    for model_index,model in enumerate(models):
        cells=model['cells']
        # Candidate camera centres are graph nodes with proven reachability.
        stride=max(1,round(.30/probe.step))
        for key,cell in sorted(cells.items()):
            # A one-cell-wide slot can fall entirely between coarse samples.
            narrow=len(model['graph'].get(key,()))<=4
            if narrow or (key[0]%stride==0 and key[1]%stride==0):
                candidates.append(dict(point=tuple(cell.point),model=model_index,key=key,targets=set()))
    if not candidates:return [],dict(target_count=0,detail_lines=0)
    candidate_tree=kd([c['point'] for c in candidates])
    targets=[]
    target_keys={}
    small_objects=set()
    object_probes=[]
    for instance in probe.depsgraph.object_instances:
        obj=instance.object
        if obj.type!='MESH' or obj.get('gs_camera_mesh_visual'):continue
        if not instance.is_instance and not obj.original.visible_get():continue
        bounds=[(instance.matrix_world@Vector(p))*probe.scale for p in obj.bound_box]
        low=Vector(tuple(min(p[j] for p in bounds) for j in range(3)))
        high=Vector(tuple(max(p[j] for p in bounds) for j in range(3)))
        diagonal=(high-low).length
        if .04<diagonal<1.5:small_objects.add(obj.original.name)
        if .04<diagonal<3.0:
            center=(low+high)*.5
            object_probes.append(center)
            for axis in range(3):
                for side in (low,high):
                    point=center.copy();point[axis]=side[axis]
                    object_probes.append(point)
    def discover(index,direction,distance):
        origin=candidates[index]['point']
        hit=probe.cast(origin,direction,distance)
        if not hit:return
        point,normal,obj=hit
        if math.dist(origin,point)<max(.20,probe.margin*1.1):return
        normal=normal.copy()
        if normal.dot(Vector(origin)-Vector(point))<0:normal.negate()
        model_index=candidates[index]['model']
        key=(model_index,obj.original.name,*(round(v/precision) for v in point),*(round(v*3) for v in normal))
        target_index=target_keys.get(key)
        if target_index is None:
            target_index=len(targets);target_keys[key]=target_index
            targets.append(dict(point=tuple(point),normal=tuple(normal),object=obj.original.name,model=model_index,
                                weight=2.0 if obj.original.name in small_objects else 1.0))
        candidates[index]['targets'].add(target_index)
    ray_count=max(48,min(128,round(64*.15/precision)))
    golden=math.pi*(3-math.sqrt(5))
    directions=[]
    for i in range(ray_count):
        z=1-2*(i+.5)/ray_count;r=math.sqrt(1-z*z)
        directions.append((r*math.cos(i*golden),r*math.sin(i*golden),z))
    for index,candidate in enumerate(candidates):
        for direction in directions:discover(index,direction,max_distance)
        if index%50==0:progress('采样墙面、家具与细部',index,len(candidates))
    # Explicit rays towards small objects reduce their chance of being missed by
    # the uniform angular sample. Actual first hits still determine the target.
    for point in object_probes:
        for _,index,distance in candidate_tree.find_n(point,min(2,len(candidates))):
            if .1<distance<=max_distance:
                direction=(point-Vector(candidates[index]['point'])).normalized()
                discover(index,direction,distance+.05)
    if not targets:return [],dict(target_count=0,detail_lines=0)
    observations={}
    visible_cache={}
    def visible(target_index,origin):
        target=targets[target_index]
        delta=Vector(target['point'])-Vector(origin)
        distance=delta.length
        if distance<max(.20,probe.margin*1.1) or distance>max_distance:return False
        if Vector(target['normal']).dot(-delta/distance)<.15:return False
        key=(target_index,tuple(round(v,4) for v in origin))
        if len(visible_cache)>200000:visible_cache.clear()
        if key not in visible_cache:
            hit=probe.cast(origin,delta/distance,distance+.02)
            visible_cache[key]=bool(hit and hit[2].original.name==target['object']
                                    and math.dist(hit[0],target['point'])<.025)
        return visible_cache[key]
    for index,target in enumerate(targets):
        views=[]
        layer_index=models[target['model']]['index']
        layer_samples=samples_by_layer[layer_index]
        neighbors=main_trees[layer_index].find_n(target['point'],min(16,len(layer_samples))) if layer_samples else []
        for _,origin_index,distance in neighbors:
            if distance>max_distance:break
            point=layer_samples[origin_index]
            if visible(index,point):
                views.append(point)
                if metrics.adequate(target['point'],views,baseline,angle):break
        observations[index]=views
        if index%400==0:progress('检查表面观察角度与视差',index,len(targets))
    already={i for i,t in enumerate(targets) if metrics.adequate(t['point'],observations[i],baseline,angle)}
    visible_cache.clear()
    ranked=layer_access.balanced_order(candidates,lambda i:sum(targets[t]['weight'] for t in candidates[i]['targets']-already),len(models))
    line_candidates=[]
    for line in gap_lines(main_routes,models,probe):
        local_targets=set()
        line_points=resample(line['points'],.15)
        for point in resample(line['points'],.35):
            for _,neighbor,_ in candidate_tree.find_range(point,.65):
                if candidates[neighbor]['model']==line['model']:local_targets.update(candidates[neighbor]['targets'])
        views={t:[p for p in line_points if visible(t,p)] for t in local_targets-already}
        if line['space_targets'] or any(metrics.adequate(targets[t]['point'],observations[t]+vs,baseline,angle) for t,vs in views.items()):
            line['observations']={t:vs for t,vs in views.items() if vs};line_candidates.append(line)
    center_keys=set()
    maximum_candidates=max(120,min(500,int(settings.contour_detail_budget)*12))
    for index in ranked:
        candidate=candidates[index]
        missing=candidate['targets']-already
        if not missing:continue
        center=candidate['point']
        center_key=(candidate['model'],*(round(v/.35) for v in center))
        if center_key in center_keys:continue
        center_keys.add(center_key)
        model=models[candidate['model']]
        valid=model['valid']
        target_index=max(missing,key=lambda t:targets[t]['weight'])
        normal=Vector(targets[target_index]['normal'])
        tangent=Vector((-normal.y,normal.x,0))
        if tangent.length<.1:tangent=Vector((1,0,0))
        tangent.normalize()
        best=None
        for direction in (tangent,Vector((-tangent.y,tangent.x,0)),Vector((1,0,0)),Vector((0,1,0))):
            for half_length in (.60,.45,.30,.225,.15):
                a=tuple(Vector(center)-direction*half_length)
                b=tuple(Vector(center)+direction*half_length)
                if not valid(a,b):continue
                # Independent short-line samples provide baseline; their camera
                # generation may later add orientations, but origins stay here.
                line_points=resample([a,b],.15)
                views={}
                local_targets=set(candidate['targets'])
                for _,neighbor,_ in candidate_tree.find_range(center,.8):
                    if candidates[neighbor]['model']==candidate['model']:
                        local_targets.update(candidates[neighbor]['targets'])
                for t in local_targets-already:
                    line_views=[p for p in line_points if visible(t,p)]
                    if line_views:views[t]=line_views
                gains=[t for t,vs in views.items() if metrics.adequate(targets[t]['point'],observations[t]+vs,baseline,angle)]
                score=sum(targets[t]['weight'] for t in gains)
                if best is None or score>best[0]:
                    best=(score,dict(points=[a,b],length=half_length*2,observations=views,
                                     model=candidate['model'],center=center))
                # Prefer a useful long baseline; shortening only helps when a
                # longer segment was geometrically blocked.
                break
        if best is not None and best[0]>0:line_candidates.append(best[1])
        if len(line_candidates)%20==0:progress('生成有收益的细部短线',len(line_candidates),maximum_candidates)
        if len(line_candidates)>=maximum_candidates:break
    selected,before,after=metrics.select_lines(targets,observations,line_candidates,
                                               int(settings.contour_detail_budget),baseline,angle,balanced=True,reserve_gaps=2)
    result=[]
    details=[]
    for candidate in selected:
        model=models[candidate['model']]
        route=planner.Route(candidate['points'],kind='DETAIL')
        route.detail_gain=candidate['gain']
        route.gap_gain=candidate.get('space_gain',0)
        route.target_objects=sorted({targets[t]['object'] for t in candidate['targets']})
        result.append((model['index'],model['label'],route))
        details.append(dict(layer=model['label'],new_surface_cells=candidate['gain'],
                            length_m=candidate['length'],target_objects=route.target_objects,kind=candidate.get('kind','DETAIL'),new_gap_cells=route.gap_gain))
    layer_reports=[]
    for i,model in enumerate(models):
        ids={j for j,t in enumerate(targets) if t['model']==i}
        layer_reports.append(dict(layer=model['label'],target_count=len(ids),main_observed=len(ids&before),
             final_observed=len(ids&after),main_ratio=len(ids&before)/max(1,len(ids)),final_ratio=len(ids&after)/max(1,len(ids)),
             detail_lines=sum(c['model']==i for c in selected),new_gap_cells=sum(c.get('space_gain',0) for c in selected if c['model']==i)))
    report=dict(target_count=len(targets),main_observed_cells=len(before),final_observed_cells=len(after),
                main_surface_ratio=len(before)/len(targets),final_surface_ratio=len(after)/len(targets),
                remaining_surface_cells=len(targets)-len(after),detail_lines=len(result),details=details,
                candidate_lines=len(line_candidates),candidate_origins=len(candidates),
                surface_cell_m=precision,minimum_baseline_m=baseline,minimum_angle_degrees=angle,
                maximum_observation_distance_m=max_distance,scope='layer_specific_sampled_candidate_observable_surfaces',
                layers=layer_reports,
                new_gap_cells=sum(c.get('space_gain',0) for c in selected),
                camera_orientations_assumed='available viewing directions; validate final camera FOV separately')
    return result,report
