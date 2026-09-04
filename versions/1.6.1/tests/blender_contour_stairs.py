import importlib.util,json,sys
from pathlib import Path
import bpy
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('blender_gs_colmap_exporter',ROOT/'__init__.py',submodule_search_locations=[str(ROOT)])
addon=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=addon
spec.loader.exec_module(addon)
addon.register()
scene=bpy.context.scene
for obj in list(scene.objects):
    bpy.data.objects.remove(obj,do_unlink=True)
def cube(name,location,size):
    bpy.ops.mesh.primitive_cube_add(size=1,location=location)
    obj=bpy.context.object
    obj.name=name
    obj.scale=size
    return obj
floors=bpy.data.collections.new('Floors')
scene.collection.children.link(floors)
floors.objects.link(cube('Lower',(2,0,-.1),(8,4,.2)))
floors.objects.link(cube('Upper',(13,0,2.9),(6,4,.2)))
cube('Roof',(7,0,6),(20,4,.2))
for i in range(1,11):
    height=.3*i
    floors.objects.link(cube(f'Step{i}',(3.7+.6*i,0,height/2),(.64,2,height)))
s=scene.gs_colmap_settings
s.floorplan_method='CONTOUR'
s.contour_detail_enabled=False
s.floorplan_layer_mode='ONE'
s.floorplan_space_mode='REACHABLE'
s.floorplan_mid_height=1.2
s.floorplan_spacing=.8
s.contour_probe_spacing=.25
s.contour_clearance=.12
s.contour_max_step=.35
s.contour_floor_collection=floors
s.contour_max_bridge=20
scene.cursor.location=(1,0,1)
bpy.context.view_layer.update()
result=addon.build_floorplan_path(scene,s)
report=result['report']
assert report['layers'][0]['component_count']==1,report
assert any(max(p.co.z for p in o.data.splines[0].points)-min(p.co.z for p in o.data.splines[0].points)>2.5
           for o in result['objects']), 'Stair route was not connected across floor heights'
first_length=report['layers'][0]['total_length']
# Centimetre scene units must produce the same physical geometry and safety distance.
for obj in list(scene.objects):
    if obj.type=='MESH':
        obj.location*=100
        obj.scale*=100
scene.cursor.location*=100
scene.unit_settings.scale_length=.01
bpy.context.view_layer.update()
scaled=addon.build_floorplan_path(scene,s)
assert abs(scaled['report']['layers'][0]['total_length']-first_length)/first_length<.05
print('STAIRS_AND_UNITS_OK',json.dumps(report),flush=True)
