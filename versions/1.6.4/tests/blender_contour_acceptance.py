import importlib.util
import json
import sys
from pathlib import Path
import bpy

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('blender_gs_colmap_exporter',ROOT/'__init__.py',submodule_search_locations=[str(ROOT)])
addon=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=addon
spec.loader.exec_module(addon)
addon.register()

def cube(name,center,size):
    bpy.ops.mesh.primitive_cube_add(size=1,location=center)
    obj=bpy.context.object
    obj.name=name
    obj.scale=size
    return obj

scene=bpy.context.scene
for obj in list(scene.objects):
    bpy.data.objects.remove(obj,do_unlink=True)
floors=bpy.data.collections.new('SupportFloors')
scene.collection.children.link(floors)
for z in (-.1,2.9):
    obj=cube('Floor',(3,2,z),(6,4,.2))
    floors.objects.link(obj)
cube('Roof',(3,2,6),(6,4,.2))
for x in (0,6):
    cube('Wall',(x,2,3),(.2,4,6))
for y in (0,4):
    cube('Wall',(3,y,3),(6,.2,6))
cube('Pillar',(3,2,3),(.6,.6,6))
cube('Sofa',(1.5,1,.45),(1.8,.8,.9))
curve=bpy.data.curves.new('Handmade','CURVE')
curve.dimensions='3D'
sp=curve.splines.new('POLY')
sp.points.add(1)
sp.points[0].co=(1,1,1,1)
sp.points[1].co=(2,1,1,1)
hand=bpy.data.objects.new('Handmade',curve)
scene.collection.objects.link(hand)
old=bpy.data.collections.new('GS_FloorPath_Auto')
scene.collection.children.link(old)

settings=scene.gs_colmap_settings
settings.floorplan_method='CONTOUR'
settings.contour_detail_enabled=False
settings.floorplan_space_mode='ALL'
settings.floorplan_layer_mode='TWO'
settings.floorplan_low_height=.4
settings.floorplan_top_height=1.6
settings.floorplan_spacing=.8
settings.contour_probe_spacing=.25
settings.contour_clearance=.12
settings.contour_floor_collection=floors
scene.cursor.location=(1,2,1)
bpy.context.view_layer.update()
result=bpy.ops.gs_colmap.auto_floorplan_path()
assert result=={'FINISHED'},result
report=json.loads(scene['gs_contour_last_report'])
assert len(report['layers'])==2,report
assert all(layer['component_count']==2 for layer in report['layers']),report
assert hand.name in scene.objects and old.name in bpy.data.collections
objects=list(settings.path_collection.all_objects)
assert objects and all(o.get('gs_explicit_capture_height') for o in objects)
for obj in objects:
    zs=[p.co.z for p in obj.data.splines[0].points]
    assert max(zs)-min(zs)<.01,(obj.name,min(zs),max(zs))

# Re-run from an upper-storey seed. It must not select the lower storey by XY.
settings.floorplan_space_mode='REACHABLE'
scene.cursor.location=(1,2,4)
first_collection=settings.path_collection
assert bpy.ops.gs_colmap.auto_floorplan_path()=={'FINISHED'}
assert first_collection.name in bpy.data.collections
assert min(p.co.z for o in settings.path_collection.all_objects for p in o.data.splines[0].points)>3
previous_coordinates=sorted([tuple(tuple(round(v,5) for v in p.co) for p in o.data.splines[0].points) for o in settings.path_collection.all_objects])
assert bpy.ops.gs_colmap.auto_floorplan_path()=={'FINISHED'}
current_coordinates=sorted([tuple(tuple(round(v,5) for v in p.co) for p in o.data.splines[0].points) for o in settings.path_collection.all_objects])
assert previous_coordinates==current_coordinates,'Existing colored preview changed rerun geometry'

# Scientific camera sampling must use the explicit curve Z, not infer new layers.
from mathutils import Vector
science=addon.scientific_planner
from types import SimpleNamespace
obj=next(iter(settings.path_collection.all_objects))
points=[Vector((1,2,4.17)),Vector((2,2,4.17)),Vector((3,2,4.17))]
old_stations,old_clear=science._adaptive_stations,science._candidate_clear
science._adaptive_stations=lambda *args: [dict(point=points[1],tangent=Vector((1,0,0)),distance=1.,step=.2,critical=False)]
science._candidate_clear=lambda *args: True
old_layers=science._layer_z_values
science._layer_z_values=lambda *args: (_ for _ in ()).throw(AssertionError('Explicit routes must not be relayered'))
cache=SimpleNamespace(settings=settings,units_per_meter=1.,horizontal_fov=1.)
try:
    groups,_,_=science._make_curve_candidate_groups(cache,[dict(object=obj,points=points)],[3],cast_rays=False)
    assert groups
    assert all(abs(c.position.z-4.17)<1e-5 for group in groups for c in group)
finally:
    science._adaptive_stations,science._candidate_clear,science._layer_z_values=old_stations,old_clear,old_layers
print('CONTOUR_ACCEPTANCE_OK',json.dumps(report),flush=True)
