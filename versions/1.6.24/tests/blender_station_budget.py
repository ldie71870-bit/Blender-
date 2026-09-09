"""A user budget is a hard ceiling even with hundreds of detail minimums."""
import importlib.util,sys
from pathlib import Path
import bpy
from mathutils import Vector
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('station_budget_test',ROOT/'__init__.py',submodule_search_locations=[str(ROOT)])
a=importlib.util.module_from_spec(spec);sys.modules[spec.name]=a;spec.loader.exec_module(a);a.register()
s=bpy.context.scene.gs_colmap_settings;s.live_update_cameras=False;s.path_count_mode='COUNT';s.camera_count=50
components=[]
for i in range(67):
    detail=i<63
    components.append(dict(object={'gs_detail_min_samples':5 if detail else 1,'gs_detail_gain':10 if detail else 0},
        points=[Vector((i%9,i//9,.3 if i%2 else 2.6)),Vector((i%9+.2,i//9,.3 if i%2 else 2.6))],length=.2 if detail else 4.))
assert sum(max(1,c['object']['gs_detail_min_samples']) for c in components)==319
for budget in (1,10,50,66,67,90,318,319,500):
    s.camera_count=budget
    result=a.path_component_counts(s,components)
    assert sum(result)==budget,(budget,sum(result))
    assert all(isinstance(n,int) and n>=0 for n in result)
    samples=a.sample_path_components_with_regions(s,components)
    assert len(samples)==budget
s.path_count_mode='DENSITY';s.path_camera_density=20;s.max_path_cameras=50
assert sum(a.path_component_counts(s,components))<=50
# Exercise actual 12-camera generation, not just the count preview.
scene=bpy.context.scene
for obj in list(scene.objects):bpy.data.objects.remove(obj,do_unlink=True)
collection=bpy.data.collections.new('Budget paths');scene.collection.children.link(collection)
for index,item in enumerate(components):
    data=bpy.data.curves.new(f'Route_{index}','CURVE');data.dimensions='3D'
    spline=data.splines.new('POLY');spline.points.add(1)
    for point,position in zip(spline.points,item['points']):point.co=(*position,1.)
    obj=bpy.data.objects.new(data.name,data);collection.objects.link(obj)
    for key,value in item['object'].items():obj[key]=value
s.path_collection=collection;s.path_count_mode='COUNT';s.camera_count=50;s.rig_mode='PATH'
s.path_capture_mode='LEGACY_PANORAMA_CUBE';s.path_station_array_mode='SPHERICAL_SHELL_12'
s.shell_radius_mode='FIXED';s.shell_radius=.05;s.shell_surface_clearance=.005;s.shell_surface_detail_enabled=False
planned=a.sample_path_components(s)
assert len(planned)==50
created=a.create_rig(scene,s)
assert len(created)==600,len(created)
assert len({c.get('station_index') for c in created})==50
print('STATION_BUDGET_50_NOT_319_OK',flush=True)
a.unregister()
