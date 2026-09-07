"""Audit sampled real surfaces against actual camera frusta and mesh occlusion."""
import math
from mathutils import Vector
from .detail_coverage import resample,kd
from . import contour_planner as planner


def camera_records(scene,cameras,scale):
    records=[]
    for camera in cameras:
        if camera.data.type!='PERSP':continue
        frame=camera.data.view_frame(scene=scene)
        xs=[v.x/-v.z for v in frame];ys=[v.y/-v.z for v in frame]
        margin_x=(max(xs)-min(xs))*.03; margin_y=(max(ys)-min(ys))*.03
        records.append((camera.matrix_world.translation*scale,camera.matrix_world.inverted(),min(xs)+margin_x,max(xs)-margin_x,min(ys)+margin_y,max(ys)-margin_y,camera.data.clip_start,camera.data.clip_end,camera.name))
    return records


def visible(probe,origin,target):
    delta=Vector(target)-Vector(origin);distance=delta.length
    if distance<.08:return False
    bvh=probe.geometry[0]
    if bvh is None:return False
    hit,normal,index,depth=bvh.ray_cast(Vector(origin),delta.normalized(),distance+.025)
    # Same object is NOT sufficient: the near side of a sofa can hide its back.
    return hit is not None and abs(depth-distance)<=.025


def observations(probe,target,records,max_distance=4.):
    origins=[]
    for origin,inverse,x0,x1,y0,y1,near,far,name in records:
        if (origin-target).length>max_distance:continue
        local=inverse@(target/probe.scale)
        if not near<=-local.z<=far:continue
        x,y=local.x/-local.z,local.y/-local.z
        if not(x0<=x<=x1 and y0<=y<=y1):continue
        if visible(probe,origin,target):origins.append(origin)
    return origins


def diverse(target,origins):
    selected=[]
    for p in origins:
        direction=(p-target).normalized()
        if all((p-q).length>=.12 and direction.dot((q-target).normalized())<=math.cos(math.radians(12)) for q in selected):selected.append(p)
        if len(selected)>=5:break
    return len(selected)


def audit_and_refine(scene,routes,probe,cameras,budget=16,progress=None):
    paths=[Vector(p) for _,_,r in routes for p in resample(r.points,.4)]
    if not paths:return [],{'sampled_surfaces':0}
    tree=kd(paths);bounds=[(min(p[j] for p in paths)-.3,max(p[j] for p in paths)+.3) for j in range(3)]
    targets={}
    # Stratify by object first so many polygons on one floor cannot consume the
    # entire audit budget. Polygon centroids remain real geometry, not free cells.
    for inst in probe.depsgraph.object_instances:
        obj=inst.object
        if obj.type!='MESH' or obj.get('gs_contour_version') or obj.get('gs_camera_mesh_visual'):continue
        if not inst.is_instance and not obj.original.visible_get():continue
        mesh=obj.to_mesh()
        if not mesh:continue
        try:
            matrix=inst.matrix_world;nm=matrix.to_3x3().inverted_safe().transposed()
            stride=max(1,len(mesh.polygons)//18)
            for face in list(mesh.polygons)[::stride][:20]:
                p=(matrix@face.center)*probe.scale
                if not all(bounds[j][0]<=p[j]<=bounds[j][1] for j in range(3)):continue
                key=tuple(round(v/.16) for v in p)
                targets.setdefault(key,(p,(nm@face.normal).normalized(),obj.original.name))
        finally:obj.to_mesh_clear()
    records=camera_records(scene,cameras,probe.scale);report=[];weak=[]
    for i,(p,n,name) in enumerate(targets.values()):
        views=observations(probe,p,records);div=diverse(p,views)
        row={'object':name,'point':tuple(p),'visible_cameras':len(views),'diverse_views':div}
        report.append(row)
        if div<3:weak.append((div,len(views),p,n,name,views,row))
        if progress and i%50==0:progress('按真实相机视锥与遮挡检查表面',i,len(targets))
    weak.sort(key=lambda x:(x[0],x[1],x[4],tuple(x[2])))
    additions=[];used=[]
    for div,count,target,normal,name,views,row in weak:
        if len(additions)>=budget:break
        if any((target-q).length<.45 for q in used):continue
        best=None
        for sign in (1.,-1.):
            for distance in (.55,.9,1.3):
                end=target+normal*(distance*sign)
                if not all(bounds[j][0]<=end[j]<=bounds[j][1] for j in range(3)):continue
                if not probe.point_clear(end) or not visible(probe,end,target):continue
                for near,_,length in tree.find_n(end,8):
                    if length<.35 or length>1.8:continue
                    start=Vector(near)
                    if not probe.segment_clear(start,end):continue
                    points=[tuple(start),tuple(end)]
                    new=[Vector(p) for p in resample(points,.15) if visible(probe,p,target)]
                    gain=diverse(target,views+new)-div
                    if gain<=0:continue
                    score=gain-.1*length
                    if best is None or score>best[0]:best=(score,points,new)
                    break
        if best:
            route=planner.Route(best[1],kind='DETAIL');route.peek_route=True;route.peek_target=tuple(target)
            route.detail_gain=1;route.gap_gain=0;route.target_objects=[name]
            additions.append((0,'SurfaceAudit',route));used.append(target)
            row['proposed_diverse_views']=diverse(target,views+best[2])
    return additions,{'sampled_surfaces':len(report),'under_three_diverse_views':len(weak),'added_routes':len(additions),'surfaces':report,
       'scope':'object-stratified polygon samples within retained path bounds; actual camera frustum and first mesh hit; not total surface coverage'}

def validate_routes(scene,settings,probe,routes,audit,progress=None,max_routes=None,peek_budget=None):
    """Keep only routes whose generated cameras improve the audited weak samples."""
    import bpy,sys
    addon=sys.modules[__package__]
    cameras=addon._generated_path_cameras(scene,settings)
    if not cameras or not audit.get('surfaces'):return routes,{'skipped':'no camera audit'}
    if settings.path_station_array_mode!='SPHERICAL_SHELL_12' or settings.path_capture_mode!='LEGACY_PANORAMA_CUBE':
        return [],{'skipped':'targeted coverage requires spherical shell camera mode'}
    weak=[r for r in audit['surfaces'] if r['diverse_views']<3]
    records=camera_records(scene,cameras,probe.scale)
    seen=[observations(probe,Vector(r['point']),records) for r in weak]
    collection=bpy.data.collections.new('GS_Coverage_Validation_Temporary');scene.collection.children.link(collection)
    objects=[];data_blocks=[];accepted=[];next_station=1000;accepted_cameras=0
    try:
        for route_index,(layer,label,route) in enumerate(routes):
            if max_routes is not None and len(accepted)>=max_routes:break
            if peek_budget is not None and label!='SurfaceAudit' and sum(l!='SurfaceAudit' for _,l,_ in accepted)>=peek_budget:continue
            if progress:progress('生成实际相机验证补线收益',route_index,len(routes))
            data=bpy.data.curves.new('GS_Validation_Target','CURVE');data_blocks.append(data)
            source=bpy.data.objects.new('GS_Validation_Target',data);objects.append(source);collection.objects.link(source)
            source['gs_peek_target']=tuple(v/probe.scale for v in route.peek_target);source['gs_route_role']='DETAIL';source['gs_contour_version']='1.6.8'
            midpoint=(Vector(route.points[0])+Vector(route.points[-1]))*.5
            local=sorted((r for i,r in enumerate(weak) if diverse(Vector(r['point']),seen[i])<3 and (Vector(r['point'])-midpoint).length<2.0 and any(visible(probe,p,Vector(r['point'])) for p in resample(route.points,.25))),key=lambda r:(r['diverse_views'],(Vector(r['point'])-midpoint).length))[:4]
            if local:
                route.peek_targets=[r['point'] for r in local]
                source['gs_peek_targets']=[v/probe.scale for r in local for v in r['point']]
            points=[Vector(p)/probe.scale for p in route.points];made=[]
            for j in range(5):
                group,*_=addon._generate_spherical_shell_12_group(scene,settings,collection,points[0].lerp(points[-1],j/4),(points[-1]-points[0]).normalized(),next_station,source,probe.depsgraph)
                next_station+=1;made.extend(group);objects.extend(group)
            bpy.context.view_layer.update()
            rec=camera_records(scene,made,probe.scale);improvements=[]
            for index,row in enumerate(weak):
                target=Vector(row['point'])
                before=min(3,diverse(target,seen[index]))
                if before>=3:continue
                new=observations(probe,target,rec)
                if min(3,diverse(target,seen[index]+new))>before:improvements.append((index,new))
            if improvements:
                accepted.append((layer,label,route))
                accepted_cameras+=len(made)
                for index,new in improvements:seen[index].extend(new)
        result={'candidate_routes':len(routes),'accepted_routes':len(accepted),'validation_cameras':accepted_cameras,'validation_stations_per_route':5,'before_under_three':len(weak),
                'after_under_three':sum(diverse(Vector(r['point']),seen[i])<3 for i,r in enumerate(weak)),
                'improved_samples':sum(diverse(Vector(r['point']),seen[i])>r['diverse_views'] for i,r in enumerate(weak)),
                'weak_samples':[dict(r,after_diverse_views=diverse(Vector(r['point']),seen[i])) for i,r in enumerate(weak)]}
    finally:
        # Defer removal until all ray casts are finished: deleting evaluated objects
        # between candidates invalidates Blender's cached dependency graph.
        for obj in tuple(collection.objects):
            if obj.type=='CAMERA':addon._remove_camera(obj)
            else:bpy.data.objects.remove(obj,do_unlink=True)
        for data in data_blocks:
            if data.users==0:bpy.data.curves.remove(data)
        bpy.data.collections.remove(collection);bpy.context.view_layer.update()
    return accepted,result
