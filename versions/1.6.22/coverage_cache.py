"""Version-independent, non-executable coverage sidecar cache (NPZ + JSON)."""
import hashlib,json,os,uuid,zipfile,io
from pathlib import Path
from array import array
import bpy
from mathutils import Vector
from mathutils.kdtree import KDTree

SCHEMA=1
NAMESPACE='gs_coverage_cache_paths_v1'
_MESSAGES={}
_HEAT=None
_RESTORE=None

def status(scene):return _MESSAGES.get(scene.as_pointer(),'')

def _config(heat,scene,settings):
    key=heat.camera_key(scene,settings);count=len(heat.cameras(scene,settings))
    return [list(key[count:]),scene.unit_settings.scale_length,
            sorted(o.name for o in settings.coverage_surface_collection.all_objects) if settings.coverage_surface_collection else None]

def _canonical(value):return json.dumps(value,ensure_ascii=False,sort_keys=True,separators=(',',':'))

def _path(scene,create=False):
    paths=bpy.app.driver_namespace.setdefault(NAMESPACE,{})
    name=paths.get(scene.as_pointer()) or scene.get('gs_coverage_cache_path')
    if name:return Path(name)
    if not create:return None
    cache_id=scene.get('gs_coverage_cache_id') or uuid.uuid4().hex
    scene['gs_coverage_cache_id']=cache_id
    root=Path(bpy.utils.user_resource('DATAFILES',path='gs_coverage_cache',create=True))
    target=root/(cache_id+'.npz');scene['gs_coverage_cache_path']=str(target)
    paths[scene.as_pointer()]=str(target)
    return target

def geometry_steps(heat,scene):
    """Hash evaluated geometry, instances and matrices, never process pointers."""
    depsgraph=bpy.context.evaluated_depsgraph_get();rows=[];mesh_digests={}
    items=[(i.object,i.matrix_world.copy(),i.is_instance) for i in depsgraph.object_instances]
    for index,(obj,matrix,instance) in enumerate(items):
        if obj.type!='MESH' or heat.region_volume.is_region(obj) or obj.get('gs_contour_version') or obj.original.get('gs_contour_version') or obj.get('gs_camera_mesh_visual'):continue
        if not instance and not obj.original.visible_get():continue
        mesh_key=obj.as_pointer()
        if mesh_key not in mesh_digests:
            mesh=obj.to_mesh()
            if mesh is None:continue
            try:
                digest=hashlib.sha256()
                for collection,attribute,width,code in ((mesh.vertices,'co',3,'f'),(mesh.loops,'vertex_index',1,'i'),(mesh.polygons,'loop_start',1,'i'),(mesh.polygons,'loop_total',1,'i'),(mesh.polygons,'normal',3,'f')):
                    values=array(code,[0])*(len(collection)*width)
                    if values:collection.foreach_get(attribute,values)
                    digest.update(values.tobytes())
                mesh_digests[mesh_key]=digest.hexdigest()
            finally:obj.to_mesh_clear()
        rows.append((obj.original.name,tuple(v for row in matrix for v in row),mesh_digests[mesh_key]))
        yield index/max(1,len(items))
    return hashlib.sha256(_canonical(sorted(rows)).encode('utf8')).hexdigest()

def _geometry(heat,scene):
    gen=geometry_steps(heat,scene)
    while True:
        try:next(gen)
        except StopIteration as done:return done.value

def save(heat,scene,st,path=None):
    """Write only an internally consistent snapshot. Errors do not erase live data."""
    try:
        if not st or st.get('dirty') or any(s.get('brush_pending') for s in st['samples']):return False
        settings=scene.gs_colmap_settings
        if st.get('update_config')!=heat.update_signature(scene,settings)[1]:return False
        old=st['camera_snapshot'];current,_=heat.update_signature(scene,settings)
        if any(current.get(name)!=value for name,value in old.items()):return False
        target=Path(path) if path else _path(scene,True)
        if not path and st.get('disk_saved_key')==st['key'] and st.get('disk_saved_samples')==len(st['samples']) and target.is_file():return True
        target.parent.mkdir(parents=True,exist_ok=True)
        _MESSAGES[scene.as_pointer()]='正在保存覆盖缓存'
        geometry=_geometry(heat,scene)
        import numpy as np
        records={r[-1]:r for r in st['records']}
        names=list(records);name_ids={name:i for i,name in enumerate(names)}
        owners=sorted({s['owner'] for s in st['samples']});owner_ids={name:i for i,name in enumerate(owners)}
        offsets=[0];view_ids=[];support_offsets=[0];support=[]
        for sample in st['samples']:
            view_ids.extend(name_ids[name] for _,name in sample['views'])
            offsets.append(len(view_ids))
            support.extend((*tuple(p),*tuple(n)) for p,n in sample.get('support',()))
            support_offsets.append(len(support))
        metadata=dict(schema=SCHEMA,geometry=geometry,config=_config(heat,scene,settings),camera_snapshot=old,camera_names=names,owners=owners,
                      saved_version='1.6.21',scene_name=scene.name)
        payload=dict(metadata=np.frombuffer(_canonical(metadata).encode('utf8'),dtype=np.uint8),
            positions=np.asarray([tuple(s['p']) for s in st['samples']],dtype=np.float64),
            normals=np.asarray([tuple(s['n']) for s in st['samples']],dtype=np.float64),
            scores=np.asarray([s['score'] for s in st['samples']],dtype=np.float64),
            observation_models=np.asarray([s.get('coverage_model',1) for s in st['samples']],dtype=np.uint8),
            support_offsets=np.asarray(support_offsets,dtype=np.int64),surface_support=np.asarray(support,dtype=np.float64).reshape((-1,6)),
            owners=np.asarray([owner_ids[s['owner']] for s in st['samples']],dtype=np.int32),
            view_offsets=np.asarray(offsets,dtype=np.int64),view_ids=np.asarray(view_ids,dtype=np.int32),
            vertices=np.asarray([(*tuple(p),*tuple(n)) for p,n,i in st['vertices']],dtype=np.float64),
            vertex_samples=np.asarray([i for p,n,i in st['vertices']],dtype=np.int32),
            triangles=np.asarray(st['triangles'],dtype=np.int32))
        temporary=target.with_suffix('.tmp')
        with temporary.open('wb') as f:np.savez_compressed(f,**payload)
        os.replace(temporary,target)
        scene['gs_coverage_cache_path']=str(target);bpy.app.driver_namespace.setdefault(NAMESPACE,{})[scene.as_pointer()]=str(target)
        st['disk_saved_key']=st['key'];st['disk_saved_samples']=len(st['samples'])
        _MESSAGES[scene.as_pointer()]='覆盖已缓存，更新插件后可恢复'
        return True
    except Exception as exc:
        _MESSAGES[scene.as_pointer()]=f'覆盖结果保留在内存，缓存保存失败：{exc}'
        return False

def restore_steps(heat,scene,settings):
    if heat.state(scene) is not None:return
    target=_path(scene)
    if not target or not target.is_file():return
    revision=heat.REVISIONS.get(scene.as_pointer(),0)
    camera_start=heat.camera_key(scene,settings)
    try:
        import numpy as np
        yield .01,'读取已保存的覆盖缓存'
        with np.load(str(target),allow_pickle=False) as data:
            meta=json.loads(data['metadata'].tobytes().decode('utf8'))
            if meta['schema']!=SCHEMA:raise ValueError('缓存格式已变化')
            if _canonical(meta['config'])!=_canonical(_config(heat,scene,settings)):raise ValueError('检测范围或参数已变化')
            current,config=heat.update_signature(scene,settings)
            old=meta['camera_snapshot']
            if any(name not in current or _canonical(current[name])!=_canonical(value) for name,value in old.items()):raise ValueError('原相机已修改或删除')
            gen=geometry_steps(heat,scene)
            while True:
                try:ratio=next(gen);yield .02+.12*ratio,'核对缓存几何（不重算相机覆盖）'
                except StopIteration as result:geometry=result.value;break
            if geometry!=meta['geometry']:raise ValueError('场景表面或遮挡几何已变化')
            yield .15,'恢复表面颜色与已有观察'
            cs=[c for c in heat.cameras(scene,settings) if c.name in old]
            scale=max(1e-9,scene.unit_settings.scale_length)
            records=heat.surface_audit.camera_records(scene,cs,scale);by_name={r[-1]:r for r in records}
            samples=[];offsets=data['view_offsets'];view_ids=data['view_ids'];names=meta['camera_names']
            positions=data['positions'];normals=data['normals'];owners=data['owners'];scores=data['scores']
            models=data['observation_models'] if 'observation_models' in data else None
            support_offsets=data['support_offsets'] if 'support_offsets' in data else None
            support=data['surface_support'] if 'surface_support' in data else None
            for i,p in enumerate(positions):
                views=[(by_name[names[j]][0],names[j]) for j in view_ids[offsets[i]:offsets[i+1]]]
                local=[(Vector(v[:3]),Vector(v[3:])) for v in support[support_offsets[i]:support_offsets[i+1]]] if support is not None and support_offsets is not None else []
                samples.append(dict(p=Vector(p),n=Vector(normals[i]),owner=meta['owners'][owners[i]],views=views,score=float(scores[i]),
                                    coverage_model=int(models[i]) if models is not None else 1,support=local))
                if i%4096==0:yield .16+.10*i/max(1,len(positions)),'恢复已缓存的表面观察'
            vertices=[(Vector(v[:3]),Vector(v[3:]),int(i)) for v,i in zip(data['vertices'],data['vertex_samples'])]
            triangles=[tuple(map(int,t)) for t in data['triangles']]
        yield .27,'恢复场景遮挡查询与笔刷索引'
        probe=heat.contour_blender.SceneProbe(scene,scale,.10,.2)
        tree=KDTree(len(samples))
        for i,s in enumerate(samples):tree.insert(s['p'],i)
        tree.balance()
        if revision!=heat.REVISIONS.get(scene.as_pointer(),0) or camera_start!=heat.camera_key(scene,settings):raise ValueError('恢复期间场景发生变化')
        snapshot={name:current[name] for name in old}
        # Baseline key intentionally excludes newly added cameras for incremental update.
        tail=camera_start[len(heat.cameras(scene,settings)):]
        key=tuple(snapshot[c.name] for c in heat.cameras(scene,settings) if c.name in snapshot)+tail
        st=dict(scale=scale,probe=probe,records=records,samples=samples,vertices=vertices,triangles=triangles,selected=set(),batch=None,dirty=False,
                key=key,scene=scene,tree=tree,camera_snapshot=snapshot,update_config=config,
                geometry_revision=heat.GEOMETRY_REVISIONS.get(scene.as_pointer(),0),last_update={'mode':'DISK_CACHE'})
        st['disk_saved_key']=key;st['disk_saved_samples']=len(samples)
        heat.STATES[scene.as_pointer()]=st;heat.CAMERA_KEYS[scene.as_pointer()]=camera_start
        _MESSAGES[scene.as_pointer()]='覆盖缓存已恢复，无需重算旧相机观察'
        settings.coverage_status=_MESSAGES[scene.as_pointer()];settings.coverage_heat_enabled=True
        heat.redraw()
        yield .30,'缓存已恢复；仅新增相机需要增量刷新'
    except Exception as exc:
        _MESSAGES[scene.as_pointer()]=f'缓存未使用：{exc}；更新覆盖将重新检测'


def _tick():
    global _RESTORE
    heat=_HEAT
    if heat is None:return None
    try:
        scene=bpy.context.scene;settings=getattr(scene,'gs_colmap_settings',None)
        if settings is None:return .5
        if _RESTORE is None:
            if heat.state(scene) is not None:return None
            settings.coverage_busy=True
            bpy.context.view_layer.update()
            _RESTORE=(scene,restore_steps(heat,scene,settings))
        saved,gen=_RESTORE
        p,t=next(gen);saved.gs_colmap_settings.coverage_status=t;heat.redraw();return .03
    except StopIteration:
        if _RESTORE:_RESTORE[0].gs_colmap_settings.coverage_busy=False
        _RESTORE=None;return None
    except Exception:
        if _RESTORE:
            try:_RESTORE[0].gs_colmap_settings.coverage_busy=False
            except ReferenceError:pass
        _RESTORE=None;return None

@bpy.app.handlers.persistent
def after_load(*args):
    if not bpy.app.background and not bpy.app.timers.is_registered(_tick):bpy.app.timers.register(_tick,first_interval=.5)

@bpy.app.handlers.persistent
def before_load(*args):
    global _RESTORE
    if bpy.app.timers.is_registered(_tick):bpy.app.timers.unregister(_tick)
    if _RESTORE:
        _RESTORE[1].close();_RESTORE=None

@bpy.app.handlers.persistent
def after_save(*args):
    if _HEAT is None:return
    for scene in bpy.data.scenes:
        st=_HEAT.state(scene)
        if st:save(_HEAT,scene,st)

def register(heat):
    global _HEAT
    _HEAT=heat
    for handlers,fn in ((bpy.app.handlers.load_pre,before_load),(bpy.app.handlers.load_post,after_load),(bpy.app.handlers.save_post,after_save)):
        if fn not in handlers:handlers.append(fn)
    after_load()

def unregister(heat):
    global _HEAT
    before_load()
    for scene in tuple(getattr(bpy.data,'scenes',())):
        st=heat.state(scene)
        if st:save(heat,scene,st)
    for handlers,fn in ((bpy.app.handlers.load_pre,before_load),(bpy.app.handlers.load_post,after_load),(bpy.app.handlers.save_post,after_save)):
        if fn in handlers:handlers.remove(fn)
    _HEAT=None
