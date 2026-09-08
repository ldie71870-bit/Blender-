import bpy,sys,importlib.util
from pathlib import Path
from mathutils import Vector
root=Path(__file__).resolve().parents[1];sp=importlib.util.spec_from_file_location('peektest',root/'__init__.py',submodule_search_locations=[str(root)]);a=importlib.util.module_from_spec(sp);sys.modules[sp.name]=a;sp.loader.exec_module(a);a.register()
from peektest import doorway_peek,surface_audit
for o in list(bpy.context.scene.objects):bpy.data.objects.remove(o,do_unlink=True)
s=bpy.context.scene.gs_colmap_settings;s.live_update_cameras=False

def cube(name,p,size):
 bpy.ops.mesh.primitive_cube_add(size=1,location=p);o=bpy.context.object;o.name=name;o.scale=size;return o
cube('Floor',(1,0,-.1),(6,6,.2));cube('Roof',(1,0,3.1),(6,6,.2))
for y in (-1.7,1.7):cube('DoorJamb',(1,y,1.5),(.2,2.4,3))
cube('RearWall',(2.8,0,1.5),(.2,6,3))
d=bpy.data.curves.new('Existing Thick Path','CURVE');d.dimensions='3D';d.bevel_depth=.04;sp=d.splines.new('POLY');sp.points.add(1);sp.points[0].co=(0,-1,1,1);sp.points[1].co=(0,1,1,1)
o=bpy.data.objects.new(d.name,d);bpy.context.scene.collection.objects.link(o);o['gs_contour_version']='test'
bpy.context.view_layer.update();probe=a.contour_blender.SceneProbe(bpy.context.scene,1.,.1,.15);probe.vertical_range=5
assert probe.point_clear((0,0,1)), 'Existing bevelled paths must not collide with themselves'
hit=a._capture_ray_cast(bpy.context.scene,probe.depsgraph,Vector((-.5,0,1)),Vector((1,0,0)),4.)
assert hit[0] and hit[4].name=='RearWall', 'Skipping a bevelled path must still detect the wall behind it'
assert a._shell_direction_radius_is_safe(bpy.context.scene,probe.depsgraph,Vector((0,0,1)),Vector((1,0,0)),.18,.03), 'A station must not reject its own bevelled source curve'
routes=[(0,'Middle',a.contour_blender.planner.Route([(0,-1,1),(0,1,1)])),(0,'Middle',a.contour_blender.planner.Route([(2.1,-1,1),(2.1,-.8,1)]))]
found,report=doorway_peek.discover(routes,probe,8,1.5)
assert found,report
assert all(probe.segment_clear(r.points[0],r.points[-1]) for _,_,r in found)
assert any(r.points[0][0]<.5 and r.points[-1][0]>1.1 for _,_,r in found),report
assert not surface_audit.visible(probe,Vector((0,1,1)),Vector((1.1,1,1))), 'Far face of same wall is occluded'
assert surface_audit.visible(probe,Vector((0,1,1)),Vector((.9,1,1)))
camera_data=bpy.data.cameras.new('AuditCamera')
camera=bpy.data.objects.new('AuditCamera',camera_data);bpy.context.scene.collection.objects.link(camera)
camera.location=(0,1,1);camera.rotation_euler=Vector((1,0,0)).to_track_quat('-Z','Y').to_euler()
bpy.context.view_layer.update()
assert surface_audit.observations(probe,Vector((.9,1,1)),surface_audit.camera_records(bpy.context.scene,[camera],1.))
camera_data.clip_start=1.1
assert not surface_audit.observations(probe,Vector((.9,1,1)),surface_audit.camera_records(bpy.context.scene,[camera],1.)), 'Near-clipped surface must not count'
camera_data.clip_start=.01;camera_data.clip_end=.5
assert not surface_audit.observations(probe,Vector((.9,1,1)),surface_audit.camera_records(bpy.context.scene,[camera],1.)), 'Far-clipped surface must not count'
camera_data.clip_end=100.;camera_data.shift_x=2.
assert not surface_audit.observations(probe,Vector((.9,1,1)),surface_audit.camera_records(bpy.context.scene,[camera],1.)), 'Lens shift must move the audited frustum'
print('DOORWAY_REAL_GEOMETRY_OK',report['accepted'],flush=True)
a.unregister()
