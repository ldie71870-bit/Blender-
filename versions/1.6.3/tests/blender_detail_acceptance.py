import importlib.util,json,sys
from pathlib import Path
import bpy
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('detail_test_addon',ROOT/'__init__.py',submodule_search_locations=[str(ROOT)])
addon=importlib.util.module_from_spec(spec);sys.modules[spec.name]=addon;spec.loader.exec_module(addon);addon.register()
scene=bpy.context.scene
for obj in list(scene.objects):bpy.data.objects.remove(obj,do_unlink=True)
def cube(name,location,size):
    bpy.ops.mesh.primitive_cube_add(size=1,location=location)
    obj=bpy.context.object;obj.name=name;obj.scale=size;return obj
floors=bpy.data.collections.new('Floors');scene.collection.children.link(floors)
floors.objects.link(cube('Floor',(2,2,-.1),(4,4,.2)))
cube('Roof',(2,2,3),(4,4,.2))
for x in (0,4):cube('Wall',(x,2,1.5),(.15,4,3))
for y in (0,4):cube('Wall',(2,y,1.5),(4,.15,3))
cube('Occluder',(2,2,1),(.5,.6,2))
cube('SmallDetail',(2,3.1,1.2),(.12,.12,.2))
bpy.context.view_layer.update()
s=scene.gs_colmap_settings;s.contour_detail_budget=4
probe=addon.contour_blender.SceneProbe(scene,1.,.12,.25)
bounds=addon.contour_blender.geometry_bounds(scene,probe);probe.vertical_range=4
supports,origin,_=addon.contour_blender.sample_supports(probe,bounds,.25,floors,lambda *a:None)
cells,graph,valid=addon.contour_blender.make_layer_graph(supports,1.2,probe,.25,.38,lambda *a:None)
main=[(0,'Middle',addon.contour_blender.planner.Route([(1,.6,1.2),(3,.6,1.2)]))]
models=[dict(index=0,label='Middle',cells=cells,graph=graph,valid=valid)]
routes,report=addon.contour_blender.detail_coverage.refine(main,models,probe,s,lambda *a:None)
assert 0<len(routes)<=4,report
assert report['final_observed_cells']>report['main_observed_cells'],report
assert all((route.detail_gain>0 or route.gap_gain>0) and all(valid(a,b) for a,b in zip(route.points,route.points[1:])) for _,_,route in routes)
assert all(.29<=addon.contour_blender.planner.length(route.points)<=4.5 for _,_,route in routes)
# A short detail curve must receive multiple camera origins even next to a long main curve.
from types import SimpleNamespace
components=[dict(object={},length=100.),dict(object={'gs_detail_min_samples':5},length=1.)]
counts=addon.path_component_counts(SimpleNamespace(path_count_mode='FIXED',camera_count=20),components)
assert counts[1]>=5 and sum(counts)==20,counts
print('DETAIL_SURFACE_ACCEPTANCE_OK',json.dumps(report),flush=True)
