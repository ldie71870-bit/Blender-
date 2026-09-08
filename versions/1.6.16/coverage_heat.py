"""Measured surface heatmap and surface-local camera repair. No scene mesh proxies."""
import math,time,sys,json
import bpy
from mathutils import Vector,Matrix
from mathutils.kdtree import KDTree
from bpy_extras import view3d_utils
from . import contour_blender, surface_audit, region_volume, coverage_patch

STATES={}
HANDLE=None
REVISIONS={}
CAMERA_KEYS={}
GEOMETRY_REVISIONS={}

def redraw():
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type=='VIEW_3D':area.tag_redraw()

def state(scene):return STATES.get(scene.as_pointer())

def palette(value):
    t=max(0.,min(1.,value))
    return (1.,2*t,0.,.86) if t<.5 else (2*(1-t),1.,0.,.86)

def cameras(scene,settings):
    pool=settings.coverage_camera_collection.all_objects if settings.coverage_camera_collection else scene.objects
    return [o for o in pool if o.type=='CAMERA' and o.data.type=='PERSP'
            and not o.get('gs_patch_preview') and not o.get('gs_heat_temporary')]

def camera_key(scene,settings):
    return tuple((c.name,tuple(v for row in c.matrix_world for v in row),c.data.lens,c.data.sensor_width,c.data.sensor_height,c.data.sensor_fit,c.data.shift_x,c.data.shift_y,c.data.clip_start,c.data.clip_end) for c in cameras(scene,settings))+tuple(getattr(settings,k) for k in ('coverage_spacing','coverage_good_views','coverage_min_distance','coverage_max_distance','coverage_incidence','coverage_baseline','coverage_angle'))+(scene.render.resolution_x,scene.render.resolution_y,scene.render.pixel_aspect_x,scene.render.pixel_aspect_y)

def update_signature(scene,settings):
    cs=cameras(scene,settings);key=camera_key(scene,settings)
    geometry=tuple((o.name,o.data.as_pointer(),tuple(v for row in o.matrix_world for v in row),o.visible_get()) for o in scene.objects if o.type=='MESH' and not o.get('gs_camera_mesh_visual') and not region_volume.is_region(o))
    scope=tuple(sorted(o.name for o in settings.coverage_surface_collection.all_objects)) if settings.coverage_surface_collection else None
    return {entry[0]:entry for entry in key[:len(cs)]},(key[len(cs):],scene.unit_settings.scale_length,scope,geometry)


def additions(scene,settings,st):
    if not st or 'update_config' not in st:return None
    current,config=update_signature(scene,settings)
    if config!=st['update_config'] or GEOMETRY_REVISIONS.get(scene.as_pointer(),0)!=st['geometry_revision']:return None
    old=st['camera_snapshot']
    if any(current.get(name)!=value for name,value in old.items()):return None
    return set(current)-set(old)


def update_coverage(scene,settings):
    bpy.context.view_layer.update()
    st=state(scene);added=additions(scene,settings,st)
    if added is None:
        yield from analysis(scene,settings);return
    if not added:
        st['dirty']=False;st['pending']=False
        yield 1.,'相机与场景未变化，已复用原覆盖结果';return
    key=camera_key(scene,settings);revision=REVISIONS.get(scene.as_pointer(),0)
    new_cameras=[c for c in cameras(scene,settings) if c.name in added]
    records=surface_audit.camera_records(scene,new_cameras,st['scale'])
    affected={}
    for rec in records:
        for _,i,_ in st['tree'].find_range(rec[0],settings.coverage_max_distance):affected.setdefault(i,[]).append(rec)
    changes={}
    yield .02,f'增量更新：{len(records)} 台新相机 / {len(affected)} 个候选表面点'
    for k,(i,recs) in enumerate(affected.items()):
        sample=st['samples'][i]
        extra=accepted_views(st,sample['p'],sample['n'],recs,settings)
        if extra:
            views=sample['views']+extra
            changes[i]=(views,len(independent(sample['p'],views,settings))/settings.coverage_good_views)
        if k%40==0:yield .02+.96*k/max(1,len(affected)),f'新相机视锥 / 遮挡检查：{k}/{len(affected)}'
    if key!=camera_key(scene,settings) or REVISIONS.get(scene.as_pointer(),0)!=revision:raise ValueError('增量检测期间场景变化，请再次更新')
    # Atomic commit: cancelling never leaves partially accumulated observations.
    for i,(views,score) in changes.items():st['samples'][i].update(views=views,score=score)
    st['records'].extend(records);st['key']=key;st['dirty']=False;st['pending']=False;st['batch']=None
    st['camera_snapshot'],st['update_config']=update_signature(scene,settings)
    CAMERA_KEYS[scene.as_pointer()]=key
    st['last_update']=dict(mode='INCREMENTAL',new_cameras=len(records),candidate_points=len(affected),changed_points=len(changes))
    yield 1.,f'增量完成：新增 {len(records)} 台相机，刷新 {len(changes)} 点，保留原表面与刷选'


def accepted_views(st,p,n,records,settings):
    result=[];limit=math.cos(math.radians(settings.coverage_incidence))
    for rec in records:
        origin,inverse,x0,x1,y0,y1,near,far,name=rec
        delta=origin-p;distance=delta.length
        if not settings.coverage_min_distance<=distance<=settings.coverage_max_distance:continue
        if n.dot(delta/distance)<limit:continue
        q=inverse@(p/st['scale']);depth=-q.z
        if depth<=0 or not near<=depth<=far:continue
        if not x0<=q.x/depth<=x1 or not y0<=q.y/depth<=y1:continue
        if surface_audit.visible(st['probe'],origin,p):result.append((origin,name))
    return result

def independent(p,views,settings):
    selected=[];limit=math.cos(math.radians(settings.coverage_angle))
    for origin,name in sorted(views,key=lambda view:view[1]):
        if all((origin-q).length>=settings.coverage_baseline and (origin-p).normalized().dot((q-p).normalized())<=limit for q,_ in selected):selected.append((origin,name))
        if len(selected)>=settings.coverage_good_views:break
    return selected

def analysis(scene,settings):
    bpy.context.view_layer.update()
    CAMERA_KEYS[scene.as_pointer()]=camera_key(scene,settings)
    revision=REVISIONS.get(scene.as_pointer(),0)
    cs=cameras(scene,settings)
    if settings.coverage_min_distance>=settings.coverage_max_distance:raise ValueError('最小观察距离必须小于最大观察距离')
    if not cs:raise ValueError('指定集合/场景没有透视相机；全景和正交相机暂不参与此检测')
    scale=max(1e-9,scene.unit_settings.scale_length)
    yield .01,'建立可见场景遮挡查询'
    probe=contour_blender.SceneProbe(scene,scale,.10,.2)
    records=surface_audit.camera_records(scene,cs,scale)
    st=dict(scale=scale,probe=probe,records=records,samples=[],vertices=[],triangles=[],selected=set(),batch=None,dirty=False,key=camera_key(scene,settings),scene=scene)
    lookup={};vertex_lookup={};allowed={o.original.name for o in settings.coverage_surface_collection.all_objects} if settings.coverage_surface_collection else None
    instances=[(i.object,i.matrix_world.copy(),i.is_instance) for i in probe.depsgraph.object_instances]
    for oi,(obj,instance_matrix,is_instance) in enumerate(instances):
        if obj.type!='MESH' or region_volume.is_region(obj) or obj.get('gs_contour_version') or obj.original.get('gs_contour_version') or obj.get('gs_camera_mesh_visual'):continue
        if allowed is not None and obj.original.name not in allowed:continue
        if not is_instance and not obj.original.visible_get():continue
        mesh=obj.to_mesh()
        if mesh is None:continue
        try:
            mesh.calc_loop_triangles();matrix=instance_matrix;nm=matrix.to_3x3().inverted_safe().transposed()
            def add(p,n):
                key=(obj.original.name,*(round(v/settings.coverage_spacing) for v in p),*(round(v*4) for v in n))
                if key not in lookup:
                    if len(st['samples'])>=300000:raise ValueError('表面采样超过 30 万点，请限定表面集合或增大采样间距')
                    lookup[key]=len(st['samples']);st['samples'].append(dict(p=p,n=n,owner=obj.original.name,views=[],score=0.))
                vertex_key=(obj.original.name,*(round(v,6) for v in p),*(round(v,3) for v in n))
                if vertex_key not in vertex_lookup:
                    vertex_lookup[vertex_key]=len(st['vertices']);st['vertices'].append((p,n,lookup[key]))
                return vertex_lookup[vertex_key]
            for tri in mesh.loop_triangles:
                a,b,c=[matrix@mesh.vertices[i].co*scale for i in tri.vertices];n=(nm@tri.normal).normalized()
                count=max(1,min(16,math.ceil(max((a-b).length,(b-c).length,(c-a).length)/settings.coverage_spacing)))
                grid={}
                for i in range(count+1):
                    for j in range(count+1-i):grid[i,j]=add(a+(b-a)*(i/count)+(c-a)*(j/count),n)
                for i in range(count):
                    for j in range(count-i):
                        st['triangles'].append((grid[i,j],grid[i+1,j],grid[i,j+1]))
                        if i+j<count-1:st['triangles'].append((grid[i+1,j],grid[i+1,j+1],grid[i,j+1]))
        finally:obj.to_mesh_clear()
        yield .05+.25*(oi+1)/max(1,len(instances)),f'采样表面：{len(st["samples"])} 点'
    if not st['samples']:raise ValueError('没有可见的待分析表面')
    tree=KDTree(len(records))
    for i,r in enumerate(records):tree.insert(r[0],i)
    tree.balance()
    for i,s in enumerate(st['samples']):
        nearby=[records[j] for _,j,_ in tree.find_range(s['p'],settings.coverage_max_distance)]
        s['views']=accepted_views(st,s['p'],s['n'],nearby,settings)
        s['score']=len(independent(s['p'],s['views'],settings))/settings.coverage_good_views
        if i%40==0:yield .30+.68*i/len(st['samples']),f'视锥 / 遮挡 / 视差：{i}/{len(st["samples"])}'
    st['tree']=KDTree(len(st['samples']))
    for i,s in enumerate(st['samples']):st['tree'].insert(s['p'],i)
    st['tree'].balance()
    if st['key']!=camera_key(scene,settings) or REVISIONS.get(scene.as_pointer(),0)!=revision:raise ValueError('检测期间场景发生变化，请重新检测')
    st['camera_snapshot'],st['update_config']=update_signature(scene,settings)
    st['geometry_revision']=GEOMETRY_REVISIONS.get(scene.as_pointer(),0)
    st['last_update']={'mode':'FULL'}
    STATES[scene.as_pointer()]=st
    yield 1.,f'完成：{len(st["samples"])} 点 / {len(records)} 台相机；红缺失 黄不足 绿充足'

def draw():
    scene=bpy.context.scene;settings=getattr(scene,'gs_colmap_settings',None);st=state(scene)
    if not settings or not settings.coverage_heat_enabled or not st or st['dirty']:return
    if st['scene']!=scene:return
    if time.monotonic()-st.get('checked',0)>1.:
        st['checked']=time.monotonic()
        if st['key']!=camera_key(scene,settings):
            if additions(scene,settings,st) is not None:
                st['pending']=True;settings.coverage_status='新增相机待增量更新，当前显示原覆盖结果'
            else:
                st['dirty']=True;settings.coverage_status='相机或检测参数变化，请重新检测';return
    import gpu
    from gpu_extras.batch import batch_for_shader
    if st['batch'] is None:
        shader=gpu.shader.from_builtin('SMOOTH_COLOR')
        positions=[tuple((p+n*.001)/st['scale']) for p,n,i in st['vertices']]
        colors=[(.1,.6,1.,.95) if i in st['selected'] else palette(st['samples'][i]['score']) for p,n,i in st['vertices']]
        st['batch']=(shader,batch_for_shader(shader,'TRIS',{'pos':positions,'color':colors},indices=st['triangles']))
    try:
        gpu.state.blend_set('ALPHA');gpu.state.depth_test_set('LESS_EQUAL');gpu.state.depth_mask_set(False)
        shader,batch=st['batch'];shader.bind();batch.draw(shader)
    finally:gpu.state.depth_mask_set(True);gpu.state.depth_test_set('NONE');gpu.state.blend_set('NONE')

def begin_stroke(st):
    if st.get('stroke_before') is None:st['stroke_before']=set(st['selected'])


def finish_stroke(st):
    before=st.pop('stroke_before',None)
    if before is None:return
    added=st['selected']-before;removed=before-st['selected']
    if added or removed:
        history=st.setdefault('brush_history',[])
        history.append((added,removed))
        del history[:-50]


def undo_stroke(st):
    finish_stroke(st)
    history=st.get('brush_history',[])
    if not history:return False
    added,removed=history.pop()
    st['selected'].difference_update(added);st['selected'].update(removed)
    st['batch']=None
    return True


def paint(st,origin,direction,radius,erase=False):
    hit,n,face,dist=st['probe'].geometry[0].ray_cast(origin,direction)
    if hit is None:return 0
    changed=0
    for _,i,d in st['tree'].find_range(hit,radius):
        s=st['samples'][i]
        # Only the visible side of the surface under the cursor, never through a wall.
        if s['n'].dot(n)<.35 or not surface_audit.visible(st['probe'],origin,s['p']):continue
        if erase:st['selected'].discard(i)
        else:st['selected'].add(i)
        changed+=1
    st['batch']=None;return changed

@bpy.app.handlers.persistent
def reset(*args):
    STATES.clear();CAMERA_KEYS.clear();REVISIONS.clear();GEOMETRY_REVISIONS.clear();GEOMETRY_REVISIONS.clear()

@bpy.app.handlers.persistent
def changed(scene,depsgraph):
    st=state(scene)
    settings=getattr(scene,'gs_colmap_settings',None)
    if settings is None:return
    relevant=False;camera_update=False
    for update in depsgraph.updates:
        obj=update.id
        if isinstance(obj,bpy.types.Camera) or (isinstance(obj,bpy.types.Object) and obj.type=='CAMERA'):
            camera_update=True
        elif isinstance(obj,bpy.types.Mesh):relevant=True
        elif isinstance(obj,bpy.types.Object) and obj.type=='MESH' and (update.is_updated_geometry or update.is_updated_transform):
            if not obj.get('gs_camera_mesh_visual') and not obj.get('gs_contour_version') and not region_volume.is_region(obj):relevant=True
    geometry_changed=relevant
    if geometry_changed:GEOMETRY_REVISIONS[scene.as_pointer()]=GEOMETRY_REVISIONS.get(scene.as_pointer(),0)+1
    if camera_update:
        current=camera_key(scene,settings)
        previous=CAMERA_KEYS.get(scene.as_pointer(),st['key'] if st else current)
        CAMERA_KEYS[scene.as_pointer()]=current
        # display_size and custom proxy metadata do not change optical coverage.
        relevant=relevant or current!=previous
    if relevant:
        REVISIONS[scene.as_pointer()]=REVISIONS.get(scene.as_pointer(),0)+1
        if st:
            if not geometry_changed and additions(scene,settings,st) is not None:
                st['pending']=True;settings.coverage_status='新增相机待增量更新，当前保留原覆盖色'
            else:st['dirty']=True;settings.coverage_status='场景已变化，请重新检测覆盖'


def register():
    global HANDLE
    if not bpy.app.background:HANDLE=bpy.types.SpaceView3D.draw_handler_add(draw,(),'WINDOW','POST_VIEW')
    bpy.app.handlers.load_pre.append(reset);bpy.app.handlers.depsgraph_update_post.append(changed)

def unregister():
    global HANDLE
    if HANDLE is not None:bpy.types.SpaceView3D.draw_handler_remove(HANDLE,'WINDOW');HANDLE=None
    for handlers,fn in ((bpy.app.handlers.load_pre,reset),(bpy.app.handlers.depsgraph_update_post,changed)):
        if fn in handlers:handlers.remove(fn)
    STATES.clear()

def repair(scene,settings,st):
    from types import SimpleNamespace
    if st['dirty'] or st['key']!=camera_key(scene,settings):raise ValueError('相机或场景已变化，请重新检测并刷选')
    ids=sorted(i for i in st['selected'] if st['samples'][i]['score']<1.)
    if not ids:raise ValueError('请先刷选欠覆盖表面；已充分覆盖的表面不会重复补拍')
    # Spatially distribute targets; a brush stroke on a large wall must not choose
    # only the first triangle or the object's origin.
    stride=max(1,math.ceil(len(ids)/64));targets=ids[::stride]
    context_ids=set(targets)
    for i in targets:
        nearby=st['tree'].find_range(st['samples'][i]['p'],settings.coverage_max_distance)
        for _,j,_ in nearby[::max(1,len(nearby)//96)]:context_ids.add(j)
    context_ids=sorted(context_ids)
    if len(context_ids)>1000:context_ids=context_ids[::math.ceil(len(context_ids)/1000)]
    context_ids=sorted(set(context_ids)|set(targets))
    cs=cameras(scene,settings);center=sum((st['samples'][i]['p'] for i in targets),Vector())/len(targets)
    reference=min(cs,key=lambda c:(c.matrix_world.translation*st['scale']-center).length)
    views={i:list(st['samples'][i]['views']) for i in context_ids}
    all_candidates=[];origins=set()
    for i in targets[::max(1,math.ceil(len(targets)/16))]:
        sample=st['samples'][i];p,n=sample['p'],sample['n'];axis=Vector((0,0,1)) if abs(n.z)<.9 else Vector((1,0,0))
        tangent=n.cross(axis).normalized();up=tangent.cross(n).normalized()
        for distance in sorted(set((max(settings.coverage_min_distance,.6),min(1.2,settings.coverage_max_distance),min(2.,settings.coverage_max_distance)))):
            for x,y in ((0,0),(.5,0),(-.5,0),(0,.4),(0,-.4)):
                origin=p+(n+tangent*x+up*y).normalized()*distance
                key=tuple(round(v/.08) for v in origin)
                if key in origins:continue
                origins.add(key);all_candidates.append((origin,p))
    probe=contour_blender.SceneProbe(scene,st['scale'],settings.coverage_safety,.2,geometry=st['probe'].geometry)
    bounds=contour_blender.geometry_bounds(scene,probe);probe.vertical_range=(bounds[1]-bounds[0]).length+2
    remaining=[]
    for ci,(origin,p) in enumerate(all_candidates):
        nearest=probe.geometry[0].find_nearest(origin,settings.coverage_safety)
        if nearest[0] is not None or not probe.point_clear(origin):continue
        rotation=(p-origin).to_track_quat('-Z','Y')
        matrix=Matrix.Translation(origin/st['scale'])@rotation.to_matrix().to_4x4()
        camera=SimpleNamespace(matrix_world=matrix,data=reference.data,name=f'brush_candidate_{ci}')
        record=surface_audit.camera_records(scene,[camera],st['scale'])[0]
        visible_ids={i for i in context_ids if accepted_views(st,st['samples'][i]['p'],st['samples'][i]['n'],[record],settings)}
        if visible_ids & set(targets):remaining.append(dict(origin=origin,matrix=matrix,record=record,visible=visible_ids))
        if ci%3==0:yield .05+.65*(ci+1)/max(1,len(all_candidates)),f'检查补拍位置：{ci+1}/{len(all_candidates)}'
    selected=[]
    while remaining and len(selected)<settings.coverage_patch_budget:
        best=None
        for candidate in remaining:
            shared={}
            for i in candidate['visible']:
                for q,name in views[i]:
                    dist=(candidate['origin']-q).length
                    a=candidate['origin']-st['samples'][i]['p'];b=q-st['samples'][i]['p']
                    ang=math.degrees(a.angle(b)) if a.length and b.length else 0
                    if dist>=settings.coverage_baseline and 2<=ang<=60:shared[name]=shared.get(name,0)+1
            overlap=max(shared.values(),default=0)/max(1,len(candidate['visible']))
            if overlap<settings.coverage_overlap:continue
            gain=0
            for i in candidate['visible'] & set(targets):
                p=st['samples'][i]['p'];before=len(independent(p,views[i],settings))
                after=len(independent(p,views[i]+[(candidate['origin'],candidate['record'][-1])],settings))
                gain+=max(0,after-before)
            if gain and (best is None or gain>best[0]):best=(gain,candidate,overlap)
        if best is None:break
        _,candidate,overlap=best;remaining.remove(candidate);candidate['overlap']=overlap;selected.append(candidate)
        for i in candidate['visible']:views[i].append((candidate['origin'],candidate['record'][-1]))
        yield .70+.25*len(selected)/settings.coverage_patch_budget,f'选择有收益补拍：{len(selected)} 台'
    # Commit only after candidate evaluation; cancel leaves original cameras intact.
    if st['dirty'] or st['key']!=camera_key(scene,settings):raise ValueError('补拍规划期间场景变化，已取消相机创建')
    created=[];collection=None
    try:
        if selected:
            collection=bpy.data.collections.get(coverage_patch.FINAL_COLLECTION)
            if collection is None:collection=bpy.data.collections.new(coverage_patch.FINAL_COLLECTION)
            if collection.name not in scene.collection.children:scene.collection.children.link(collection)
        for candidate in selected:
            data=reference.data.copy();data.dof.use_dof=False;obj=bpy.data.objects.new('cam_brush_patch',data);created.append(obj)
            collection.objects.link(obj);obj.matrix_world=candidate['matrix'];obj['gs_patch_camera']=True
            obj['gs_brush_patch']=True;obj['gs_brush_overlap']=candidate['overlap'];obj.data.display_size=.08/st['scale']
            obj['gs_dataset_image_stem']=obj.name.replace('.','_');obj.hide_render=False
        scene['gs_brush_report']=json.dumps(dict(created=len(created),target_samples=len(targets),painted_samples=len(ids),reference=reference.name,
            overlap_min=settings.coverage_overlap,baseline_m=settings.coverage_baseline,distance_m=[settings.coverage_min_distance,settings.coverage_max_distance],safety_m=settings.coverage_safety))
    except Exception:
        for obj in created:
            data=obj.data;bpy.data.objects.remove(obj,do_unlink=True)
            if data.users==0:bpy.data.cameras.remove(data)
        raise
    if selected:st['pending']=True
    yield 1.,f'新增 {len(created)} 台正式补拍相机；点击更新覆盖进行增量刷新' if created else '没有满足净空、重叠和新增视差的补拍位置；未创建无效相机'

class GS_OT_heat_job(bpy.types.Operator):
    bl_idname='gs_colmap.coverage_heat_job';bl_label='检测场景覆盖'
    action:bpy.props.EnumProperty(items=(('ANALYZE','检测覆盖',''),('REPAIR','生成刷选补拍相机','')))
    def start(self,context):
        s=context.scene.gs_colmap_settings
        if s.coverage_busy:raise ValueError('覆盖任务正在运行')
        self.scene=context.scene;self.settings=s
        self.iterator=update_coverage(self.scene,s) if self.action=='ANALYZE' else repair(self.scene,s,state(self.scene))
        s.coverage_progress=0.;s.coverage_busy=True
    def execute(self,context):
        try:
            self.start(context)
            for p,t in self.iterator:self.settings.coverage_progress=p;self.settings.coverage_status=t
            self.settings.coverage_heat_enabled=True
            return {'FINISHED'}
        except Exception as e:self.report({'ERROR'},str(e));return {'CANCELLED'}
        finally:context.scene.gs_colmap_settings.coverage_busy=False
    def invoke(self,context,event):
        if bpy.app.background:return self.execute(context)
        try:self.start(context)
        except Exception as e:self.report({'ERROR'},str(e));return {'CANCELLED'}
        self.timer=context.window_manager.event_timer_add(.05,window=context.window);context.window_manager.modal_handler_add(self);return {'RUNNING_MODAL'}
    def modal(self,context,event):
        if event.type=='ESC':
            self.iterator.close();self.settings.coverage_status='已取消';self.finish(context);return {'CANCELLED'}
        if event.type!='TIMER':return {'PASS_THROUGH'}
        try:
            p,t=next(self.iterator);self.settings.coverage_progress=p;self.settings.coverage_status=t;redraw()
        except StopIteration:
            self.settings.coverage_heat_enabled=True;self.finish(context);return {'FINISHED'}
        except Exception as e:
            self.settings.coverage_status=str(e);self.report({'ERROR'},str(e));self.finish(context);return {'CANCELLED'}
        return {'PASS_THROUGH'}
    def finish(self,context):
        context.window_manager.event_timer_remove(self.timer);self.settings.coverage_busy=False;redraw()

class GS_OT_coverage_brush(bpy.types.Operator):
    bl_idname='gs_colmap.coverage_brush';bl_label='刷选欠覆盖表面'
    def invoke(self,context,event):
        st=state(context.scene)
        if context.area.type!='VIEW_3D' or not st or st['dirty']:
            self.report({'ERROR'},'请在三维视图先检测覆盖');return {'CANCELLED'}
        context.scene.gs_colmap_settings.coverage_heat_enabled=True
        self.area=context.area;self.region=next(r for r in context.area.regions if r.type=='WINDOW');self.dragging=False
        context.window_manager.modal_handler_add(self);context.window.cursor_modal_set('PAINT_BRUSH')
        self.area.header_text_set('左键刷选 · Shift 擦除 · Ctrl+Z 撤回笔画 · Enter / Esc 完成')
        return {'RUNNING_MODAL'}
    def modal(self,context,event):
        st=state(context.scene)
        if event.type in {'ESC','RET','NUMPAD_ENTER'} or not st or st['dirty']:
            if st:finish_stroke(st)
            context.window.cursor_modal_restore();self.area.header_text_set(None);return {'FINISHED'}
        if event.type=='Z' and event.ctrl and event.value=='PRESS':
            undo_stroke(st);self.dragging=False;redraw();return {'RUNNING_MODAL'}
        if event.type=='LEFTMOUSE':
            if event.value=='RELEASE':
                was_dragging=self.dragging;self.dragging=False;finish_stroke(st)
                return {'RUNNING_MODAL'} if was_dragging else {'PASS_THROUGH'}
            if event.value=='PRESS':
                x,y=event.mouse_x-self.region.x,event.mouse_y-self.region.y
                if not (0<=x<self.region.width and 0<=y<self.region.height):return {'PASS_THROUGH'}
                begin_stroke(st);self.dragging=True
        if self.dragging and event.type in {'LEFTMOUSE','MOUSEMOVE'}:
            xy=(event.mouse_x-self.region.x,event.mouse_y-self.region.y)
            if 0<=xy[0]<self.region.width and 0<=xy[1]<self.region.height:
                rv3d=self.area.spaces.active.region_3d
                origin=view3d_utils.region_2d_to_origin_3d(self.region,rv3d,xy)*st['scale']
                direction=view3d_utils.region_2d_to_vector_3d(self.region,rv3d,xy)
                paint(st,origin,direction,context.scene.gs_colmap_settings.coverage_brush_radius,event.shift);redraw()
            return {'RUNNING_MODAL'}
        if event.type=='LEFTMOUSE':return {'RUNNING_MODAL'}
        return {'PASS_THROUGH'}

class GS_OT_undo_brush(bpy.types.Operator):
    bl_idname='gs_colmap.undo_coverage_brush';bl_label='撤回上一步刷选'
    bl_description='撤回上一整笔刷选或擦除，保留更早的选择（最多 50 步）'
    @classmethod
    def poll(cls,context):
        st=state(context.scene);settings=context.scene.gs_colmap_settings
        return bool(st and not st['dirty'] and not settings.coverage_busy and (st.get('brush_history') or st.get('stroke_before') is not None))
    def execute(self,context):
        st=state(context.scene)
        if not st or not undo_stroke(st):return {'CANCELLED'}
        redraw();return {'FINISHED'}

CLASSES=(GS_OT_heat_job,GS_OT_coverage_brush,GS_OT_undo_brush)


def draw_ui(layout,context,settings):
    box=layout.box();box.label(text='相机覆盖与笔刷补拍')
    box.prop(settings,'coverage_heat_enabled',text='显示覆盖颜色')
    box.label(text='红：缺失　黄：不足　绿：充足　蓝：刷选')
    row=box.row();row.enabled=not settings.coverage_busy
    row.operator('gs_colmap.coverage_heat_job',text='检测 / 更新覆盖').action='ANALYZE'
    box.prop(settings,'coverage_progress',text='覆盖 / 补拍进度',slider=True)
    if settings.coverage_status:box.label(text=settings.coverage_status[:90])
    st=state(context.scene)
    box.prop(settings,'coverage_brush_radius',text='笔刷半径 (m)')
    row=box.row();row.enabled=bool(st and not st['dirty'] and not settings.coverage_busy)
    row.operator('gs_colmap.coverage_brush',text='开始刷选');row.operator('gs_colmap.undo_coverage_brush',text='撤回上一步',icon='LOOP_BACK')
    row=box.row();row.enabled=bool(st and st['selected'] and not st['dirty'] and not settings.coverage_busy)
    row.operator('gs_colmap.coverage_heat_job',text='为刷选区域生成补拍相机').action='REPAIR'
    if st:box.label(text=f'已刷选 {len(st["selected"])} 个表面采样点')
    if settings.interface_mode=='PRO':
        for key in ('coverage_camera_collection','coverage_surface_collection','coverage_spacing','coverage_good_views','coverage_min_distance','coverage_max_distance','coverage_incidence','coverage_baseline','coverage_angle','coverage_safety','coverage_overlap','coverage_patch_budget'):box.prop(settings,key)
