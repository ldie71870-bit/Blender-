import importlib.util,json,sys,time
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
# A saved legacy budget below the detail minimum expands instead of aborting
# the entire spherical-shell build during its prepare phase.
counts=addon.path_component_counts(SimpleNamespace(path_count_mode='FIXED',camera_count=2),components)
assert counts==[1,5],counts

# One blocked shell direction must not discard the other cameras at a station.
shell_collection=bpy.data.collections.new('ShellPartial');scene.collection.children.link(shell_collection)
s.shell_radius_mode='FIXED';s.shell_radius=.18;s.shell_surface_clearance=.03;s.shell_station_rotation_step=5
route_data=bpy.data.curves.new('VisibleRoute','CURVE');route_data.dimensions='3D';route_data.bevel_depth=.02
route_spline=route_data.splines.new('POLY');route_spline.points.add(1)
route_spline.points[0].co=(1.5,2,.1,1);route_spline.points[1].co=(2.5,2,.1,1)
visible_route=bpy.data.objects.new('VisibleRoute',route_data);scene.collection.objects.link(visible_route)
visible_route['gs_contour_version']='test';visible_route['gs_route_role']='MAIN';bpy.context.view_layer.update()
group,radii,used_fallback,skipped,_=addon._generate_spherical_shell_12_group(
    scene,s,shell_collection,(2,2,.1),(0,1,0),1,visible_route,bpy.context.evaluated_depsgraph_get())
assert 0<len(group)<12,(len(group),radii,used_fallback,skipped)
assert len(group)==len(radii) and not used_fallback and not skipped
directions_0,_=addon._world_shell_directions((0,1,0),0)
directions_5,_=addon._world_shell_directions((0,1,0),s.shell_station_rotation_step)
assert any((a-b).length>1e-3 for a,b in zip(directions_0,directions_5))

# Exercise the same sliced station state machine used by the foreground button.
staging=bpy.data.collections.new('ShellIncremental');scene.collection.children.link(staging)
op=SimpleNamespace(_sampled=[((2,2,.1),(0,1,0),visible_route)],_staging=staging,
    _cameras=[],_debug_specs=[],_actual_radii=[],_index=0,_adaptive_shrunk=0,
    _fallback_count=0,_skipped_count=0,_filtered_camera_count=0,
    _requested_radius=s.shell_radius,_depsgraph=bpy.context.evaluated_depsgraph_get(),
    _generation_started=time.monotonic(),_station_state=None)
for method in ('_set_progress','_start_position_probe','_begin_station_probe','_mark_final_names',
               '_record_station_result','_create_safe_shell_camera','_finish_unsafe_station',
               '_advance_shell_direction','_advance_shell_candidate','_position_probe_passed',
               '_generate_station_slice'):
    setattr(op,method,getattr(addon.GSCOLMAP_OT_create_cameras,method).__get__(op))
op._tag_redraw=addon.GSCOLMAP_OT_create_cameras._tag_redraw
for _ in range(1000):
    if op._index:break
    op._generate_station_slice(bpy.context)
assert op._index==1 and 0<len(op._cameras)<12,(op._index,len(op._cameras))
assert op._filtered_camera_count==12-len(op._cameras)
print('DETAIL_SURFACE_ACCEPTANCE_OK',json.dumps(report),flush=True)
