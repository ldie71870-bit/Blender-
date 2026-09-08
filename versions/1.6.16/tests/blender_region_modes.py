import bpy,sys,importlib.util
from pathlib import Path
root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('region_mode_test',root/'__init__.py',submodule_search_locations=[str(root)])
a=importlib.util.module_from_spec(spec);sys.modules[spec.name]=a;spec.loader.exec_module(a);a.register()
scene=bpy.context.scene;s=scene.gs_colmap_settings;s.live_update_cameras=False
s.interface_mode='PRO';assert bpy.ops.gs_colmap.create_path_region()=={'FINISHED'}
box=s.contour_region_object
assert a.region_volume.active_region(scene,s) is not None
s.interface_mode='SIMPLE'
assert s.contour_region_enabled and s.contour_region_object==box
assert a.region_volume.active_region(scene,s) is None
s.floorplan_method='GRID';a.prepare_contour_region(bpy.context)
s.interface_mode='PRO';assert a.region_volume.active_region(scene,s) is not None
s.interface_mode='SIMPLE';s.contour_region_object=None
a.prepare_contour_region(bpy.context)
assert a.region_volume.active_region(scene,s) is None
s.interface_mode='PRO'
try:a.region_volume.active_region(scene,s)
except ValueError:pass
else:raise AssertionError('Professional mode must validate missing region')
print('REGION_MODE_OK: simple ignores active/missing box; pro preserves and validates box')
