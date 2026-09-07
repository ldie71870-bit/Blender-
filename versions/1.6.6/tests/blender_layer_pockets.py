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
cube('Roof',(3,2,3),(6,4,.2))
cube('Sofa divider',(3,2,.45),(.7,4,.9))
cube('Low ceiling over landing',(4.7,2,2.1),(2.6,4,.2))
bpy.context.view_layer.update()
s.floorplan_space_mode='REACHABLE';s.contour_min_area=.1;s.contour_adapt_top=True
s.floorplan_mid_height=1.2;s.floorplan_ceiling_offset=.3
probe=addon.contour_blender.SceneProbe(scene,1,.12,.2);probe.vertical_range=4
bounds=addon.contour_blender.geometry_bounds(scene,probe)
supports,_,_=addon.contour_blender.sample_supports(probe,bounds,.2,floors,lambda *a:None)
models=addon.contour_blender.build_models(supports,[('Low',.3),('Middle',1.2),('High',2.0)],probe,.2,s,(1,2,.5),lambda *a:None)
assert any(c.point[0]>4 for c in models[0]['cells'].values()),'Low pocket behind sofa was dropped'
assert models[0]['component_count']>=2,'Fixture must split low traversal'
adapted=[c for c in models[2]['cells'].values() if c.point[0]>4]
assert adapted and all(1.35<c.point[2]<1.8 for c in adapted),'Top camera vanished under landing ceiling'
assert all(probe.point_clear(c.point) for c in adapted)
# Crossing to the upper view must not permit passage through the ceiling itself.
assert not probe.segment_clear((4.5,2,1.7),(4.5,2,2.5))
main=[(1,'Middle',addon.contour_blender.planner.Route([(1,1,1.2),(1,3,1.2)]))]
s.contour_detail_budget=6
routes,report=addon.contour_blender.detail_coverage.refine(main,models,probe,s,lambda *a:None)
by_layer={layer['layer']:layer for layer in report['layers']}
assert by_layer['Low']['main_observed']==0 and by_layer['High']['main_observed']==0,'Middle views incorrectly filled another height'
assert by_layer['Low']['final_observed']>0 and by_layer['High']['final_observed']>0,report
assert by_layer['Low']['detail_lines']>0 and by_layer['High']['detail_lines']>0,report
print('LOW_POCKET_AND_ADAPTIVE_TOP_OK', [len(m['cells']) for m in models],flush=True)
