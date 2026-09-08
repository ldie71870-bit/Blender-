import bpy,sys,importlib.util,math
from pathlib import Path
from mathutils import Vector
root=Path(__file__).resolve().parents[1];spec=importlib.util.spec_from_file_location('heat_test',root/'__init__.py',submodule_search_locations=[str(root)])
a=importlib.util.module_from_spec(spec);sys.modules[spec.name]=a;spec.loader.exec_module(a);a.register();h=a.coverage_heat
scene=bpy.context.scene;s=scene.gs_colmap_settings;s.live_update_cameras=False
for obj in list(scene.objects):bpy.data.objects.remove(obj,do_unlink=True)
def plane(name,y):
 mesh=bpy.data.meshes.new(name);mesh.from_pydata([(-1,y,0),(1,y,0),(1,y,2),(-1,y,2)],[],[(0,1,2,3)]);mesh.update();o=bpy.data.objects.new(name,mesh);scene.collection.objects.link(o);return o
front=plane('Front',0);back=plane('Back',.6)
surfaces=bpy.data.collections.new('Surfaces');scene.collection.children.link(surfaces);surfaces.objects.link(front);surfaces.objects.link(back);s.coverage_surface_collection=surfaces
for z in (-.2,3.):
 bpy.ops.mesh.primitive_cube_add(size=1,location=(0,-1,z));bpy.context.object.scale=(8,8,.2)

for i,x in enumerate((-.7,0,.7)):
 data=bpy.data.cameras.new('cam');data.lens=24;o=bpy.data.objects.new('cam',data);scene.collection.objects.link(o);o.location=(x,-2,1);o.rotation_euler=(Vector((0,0,1))-o.location).to_track_quat('-Z','Y').to_euler()
bpy.context.view_layer.update()
s.coverage_spacing=.5
for p,t in h.analysis(scene,s):pass
st=h.state(scene)
frontsamples=[v for v in st['samples'] if v['owner']=='Front'];backsamples=[v for v in st['samples'] if v['owner']=='Back']
assert any(v['score']>=1 for v in frontsamples),[(v['p'][:],v['score']) for v in frontsamples]
assert all(v['score']==0 for v in backsamples),'Occluded back surface falsely covered'
assert h.palette(0)==(1.,0.,0.,.86) and h.palette(1)==(0.,1.,0.,.86)
h.paint(st,Vector((0,-2,1)),Vector((0,1,0)),.8)
assert st['selected'] and all(st['samples'][i]['owner']=='Front' for i in st['selected'])
# Same optical centre with different camera names is one independent observation.
p=Vector((0,0,1));assert len(h.independent(p,[(Vector((0,-2,1)),'a'),(Vector((0,-2,1)),'b')],s))==1
# Deliberately raise target quality so brushed surface needs an extra view.
s.coverage_good_views=5
for p,t in h.analysis(scene,s):pass
st=h.state(scene);h.paint(st,Vector((0,-2,1)),Vector((0,1,0)),.8)
s.coverage_patch_budget=4

for p,t in h.repair(scene,s,st):pass
samples=st['samples'];painted=set(st['selected']);old_probe=st['probe']
for p,t in h.update_coverage(scene,s):pass
inc=h.state(scene)
assert inc is st and inc['samples'] is samples and inc['probe'] is old_probe
assert inc['selected']==painted
assert inc['last_update']['mode']=='INCREMENTAL',inc['last_update']
counts=[len(v['views']) for v in samples];scores=[v['score'] for v in samples]
for p,t in h.update_coverage(scene,s):pass
assert counts==[len(v['views']) for v in samples],'Repeated refresh duplicated observations'
for p,t in h.analysis(scene,s):pass
full=h.state(scene)
assert scores==[v['score'] for v in full['samples']],(scores,[v['score'] for v in full['samples']])
assert counts==[len(v['views']) for v in full['samples']]
cam=h.cameras(scene,s)[0];cam.data.lens+=1;bpy.context.view_layer.update()
assert h.additions(scene,s,full) is None
for p,t in h.update_coverage(scene,s):pass
assert h.state(scene)['last_update']['mode']=='FULL'
print('INCREMENTAL_EQUIVALENCE_OK',inc['last_update'],flush=True)
a.unregister()
