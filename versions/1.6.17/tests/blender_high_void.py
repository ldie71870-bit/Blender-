"""Furniture divides the low layer; top camera adjusts under a lower ceiling."""
import bpy,importlib.util,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('pocket_test_addon',ROOT/'__init__.py',submodule_search_locations=[str(ROOT)])
addon=importlib.util.module_from_spec(spec);sys.modules[spec.name]=addon;spec.loader.exec_module(addon);addon.register()
for obj in list(bpy.context.scene.objects):bpy.data.objects.remove(obj,do_unlink=True)
scene=bpy.context.scene;s=scene.gs_colmap_settings
def cube(name,location,size):
    bpy.ops.mesh.primitive_cube_add(size=1,location=location)
    obj=bpy.context.object;obj.name=name;obj.scale=size;return obj
floors=bpy.data.collections.new('Floors');scene.collection.children.link(floors)
floors.objects.link(cube('Floor',(3,2,-.1),(6,4,.2)))
cube('Roof',(3,2,6),(6,4,.2))
cube('Sofa divider',(3,2,.45),(.7,4,.9))
cube('Low ceiling over landing',(4.7,2,2.1),(2.6,4,.2))
bpy.context.view_layer.update()
s.floorplan_space_mode='REACHABLE';s.contour_min_area=.1;s.contour_adapt_top=True
s.floorplan_mid_height=1.2;s.floorplan_ceiling_offset=.3
probe=addon.contour_blender.SceneProbe(scene,1,.12,.2);probe.vertical_range=8
bounds=addon.contour_blender.geometry_bounds(scene,probe)
supports,_,_=addon.contour_blender.sample_supports(probe,bounds,.2,floors,lambda *a:None)
models=addon.contour_blender.build_models(supports,[('Low',.3),('Middle',1.2),('High',2.0),('Ceiling',2.0)],probe,.2,s,(1,2,.5),lambda *a:None)
main=[(1,'Middle',addon.contour_blender.planner.Route([(1,1,1.2),(1,3,1.2)]))]
s.contour_detail_budget=6
routes,report=addon.contour_blender.detail_coverage.refine(main,models,probe,s,lambda *a:None)
by_layer={layer['layer']:layer for layer in report['layers']}
assert by_layer['Low']['main_observed']==0 and by_layer['High']['main_observed']==0,'Middle views incorrectly filled another height'
assert by_layer['Low']['final_observed']>0,report
assert by_layer['Low']['detail_lines']>0,report
assert models[3]['cells'] and max(c.point[2] for c in models[3]['cells'].values())>5.4
assert by_layer['Ceiling']['detail_lines']>0,report
s.interface_mode='SIMPLE';s.contour_detail_enabled=False;s.contour_detail_budget=8
policy=addon.contour_blender.effective_planning_settings(s)
assert policy.contour_detail_enabled and policy.contour_detail_budget>=48
assert not s.contour_detail_enabled and s.contour_detail_budget==8
s.interface_mode='PRO';assert not addon.contour_blender.effective_planning_settings(s).contour_detail_enabled
print('LOW_POCKET_AND_ADAPTIVE_TOP_OK', [len(m['cells']) for m in models],flush=True)
