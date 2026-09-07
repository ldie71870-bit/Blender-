"""Real mesh region constraints, editable topology and background round trip."""
import importlib.util
import json
import math
import sys
import time
from pathlib import Path

import bpy
import bmesh
from mathutils import Vector

root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('region_test',root/'__init__.py',submodule_search_locations=[str(root)])
addon=importlib.util.module_from_spec(spec);sys.modules[spec.name]=addon;spec.loader.exec_module(addon);addon.register()
scene=bpy.context.scene
for obj in list(scene.objects):bpy.data.objects.remove(obj,do_unlink=True)
settings=scene.gs_colmap_settings
settings.live_update_cameras=False
assert not settings.contour_region_enabled
assert addon._addon_version_str()=='1.6.9'
metadata=addon.bl_info
addon.bl_info={}
assert addon._addon_version_str()=='1.6.9', 'Extension metadata must not reset the UI to 1.3.4'
addon.bl_info=metadata

def cube(name,location,dimensions):
    bpy.ops.mesh.primitive_cube_add(size=1,location=location)
    obj=bpy.context.object;obj.name=name;obj.scale=dimensions
    return obj

floor=cube('Floor',(2,2,-.1),(8,8,.2))
cube('Ceiling',(2,2,3.1),(8,8,.2))
floors=bpy.data.collections.new('Floors');scene.collection.children.link(floors);floors.objects.link(floor)
scene.cursor.location=(2,2,1.5)
assert bpy.ops.gs_colmap.create_path_region()=={'FINISHED'}
box=settings.contour_region_object
assert box.type=='MESH' and box.display_type=='WIRE' and box.hide_render
assert (box.location-scene.cursor.location).length<1e-6
box.rotation_euler.z=.43;box.scale=(-1.2,.7,1.)
bpy.context.view_layer.update()
volume=addon.region_volume.active_region(scene,settings)
assert volume.contains(box.matrix_world@Vector((0,0,0)))
assert not volume.contains(box.matrix_world@Vector((3,0,0)))
assert bpy.ops.gs_colmap.edit_path_region()=={'FINISHED'}
bm=bmesh.from_edit_mesh(box.data)
bmesh.ops.subdivide_edges(bm,edges=list(bm.edges),cuts=1,use_grid_fill=True)
bmesh.update_edit_mesh(box.data)
addon.prepare_contour_region(bpy.context)
assert box.mode=='OBJECT'
assert bpy.ops.gs_colmap.validate_path_region()=={'FINISHED'}

# Replace the box with an extruded L, whose missing corner lies inside its AABB.
outline=[(.3,.3),(3.7,.3),(3.7,1.5),(1.5,1.5),(1.5,3.7),(.3,3.7)]
vertices=[(x,y,z) for z in (.2,2.6) for x,y in outline]
n=len(outline)
faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]
faces += [(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
data=bpy.data.meshes.new('Concave Region');data.from_pydata(vertices,[],faces);data.update()
shape=bpy.data.objects.new('Concave Region',data);scene.collection.objects.link(shape)
settings.contour_region_object=shape
bpy.context.view_layer.update()
volume=addon.region_volume.active_region(scene,settings)
assert volume.contains((3,1,1)) and volume.contains((1,3,1))
assert not volume.contains((3,3,1))
assert not volume.segment_inside((3,1,1),(1,3,1)), 'Concave crossing must not pass just because both endpoints are inside'
assert volume.segment_inside((.6,.6,1),(.6,3.3,1)), (volume.contains((.6,.6,1)),volume.contains((.6,3.3,1)),volume.contains((.6,1.95,1)),volume.intersections((.6,.6,1),(0,1,0),2.7))
probe=addon.contour_blender.SceneProbe(scene,1.,.12,.25,region=volume);probe.vertical_range=5
assert probe.point_clear((.6,.6,1))
assert not probe.point_clear((3,3,1))
assert not probe.segment_clear((3,1,1),(1,3,1))
hit=addon._capture_ray_cast(scene,probe.depsgraph,Vector((.6,.6,1)),Vector((0,0,1)),5.)
assert hit[0] and hit[4].name=='Ceiling', 'All region helpers must be ignored by camera rays'
from types import SimpleNamespace
cache=SimpleNamespace(scene=scene,depsgraph=probe.depsgraph)
cache.ray_bvh,cache.ray_faces=addon.scientific_planner._build_scene_bvh(cache)
hit=addon.scientific_planner._ray_cast(cache,Vector((.6,.6,1)),Vector((0,0,1)),5.)
assert hit[0] and hit[4].name=='Ceiling','Scientific camera BVH must ignore region helpers'
settings.contour_floor_collection=floors
settings.floorplan_layer_mode='TWO';settings.floorplan_low_height=.4;settings.floorplan_top_height=2.
settings.contour_probe_spacing=.25;settings.contour_clearance=.12;settings.floorplan_spacing=.7
settings.contour_min_area=.1;settings.contour_detail_budget=2
settings.contour_peek_budget=2;settings.contour_camera_audit_enabled=False
settings.floorplan_space_mode='REACHABLE'
scene.cursor.location=(100,100,100)
before={o.name for o in scene.objects}
assert bpy.ops.gs_colmap.auto_floorplan_path()=={'FINISHED'}
assert before <= {o.name for o in scene.objects}
paths=list(settings.path_collection.all_objects)
assert paths
for path in paths:
    points=addon.curve_polyline_points(path,12)
    assert all(volume.segment_inside(a,b) for a,b in zip(points,points[1:])),path.name
assert json.loads(scene['gs_contour_last_report'])['region']==shape.name

# An open shape fails before any preview objects are committed.
bm=bmesh.new();bm.from_mesh(shape.data)
bm.faces.ensure_lookup_table();bmesh.ops.delete(bm,geom=[bm.faces[0]],context='FACES')
bm.to_mesh(shape.data);bm.free();bpy.context.view_layer.update()
try:addon.region_volume.active_region(scene,settings)
except ValueError as exc:assert '封闭' in str(exc)
else:raise AssertionError('Open mesh accepted')
assert settings.path_collection and all(p.name in scene.objects for p in paths)
settings.contour_region_enabled=False
assert addon.region_volume.active_region(scene,settings) is None

# Restore the closed mesh and test the real subprocess's serialized pointer.
shape.data.clear_geometry();shape.data.from_pydata(vertices,[],faces);shape.data.update()
settings.contour_region_enabled=True
settings.contour_detail_enabled=False;settings.contour_peek_enabled=False
bpy.context.view_layer.update()
job=addon.contour_jobs.start(scene,addon.__file__)
try:
    deadline=time.monotonic()+120
    while job['process'].poll() is None and time.monotonic()<deadline:time.sleep(.1)
    assert job['process'].poll() is not None,'Region worker timeout'
    result=addon.contour_jobs.finish(job)
    assert result['report']['region']==shape.name
    for obj in settings.path_collection.all_objects:
        points=addon.curve_polyline_points(obj,12)
        assert all(volume.segment_inside(a,b) for a,b in zip(points,points[1:]))
finally:addon.contour_jobs.dispose(job)
print('REGION_VOLUME_OK',json.dumps(dict(routes=len(paths),background_routes=result['report']['route_count'],ui_version=addon._addon_version_str())),flush=True)
addon.unregister()
