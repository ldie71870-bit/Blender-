"""Acceptance checks for optional rectangular station quotas."""
import bpy
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "station_region_test_addon", ROOT / "__init__.py",
    submodule_search_locations=[str(ROOT)],
)
addon = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = addon
spec.loader.exec_module(addon)
addon.register()

scene = bpy.context.scene
for obj in list(scene.objects):
    bpy.data.objects.remove(obj, do_unlink=True)
paths = bpy.data.collections.new("Station Test Paths")
scene.collection.children.link(paths)


def line(name, y):
    data = bpy.data.curves.new(name + " Data", "CURVE")
    data.dimensions = "3D"
    spline = data.splines.new("POLY")
    spline.points.add(1)
    spline.points[0].co = (0.0, y, 1.0, 1.0)
    spline.points[1].co = (10.0, y, 1.0, 1.0)
    obj = bpy.data.objects.new(name, data)
    paths.objects.link(obj)
    return obj


first = line("First Path", 0.0)
line("Second Path", 5.0)
settings = scene.gs_colmap_settings
settings.live_update_cameras = False
settings.rig_mode = "PATH"
settings.path_collection = paths
settings.path_count_mode = "COUNT"
settings.camera_count = 10

# The feature is opt-in and must leave the established sampler byte-for-byte equivalent in count.
assert not settings.station_plan_enabled
assert len(addon.sample_path_components(settings)) == 10

region = settings.station_regions.add()
region.name = "Sofa / table gap"
region.x_min, region.x_max = 2.0, 4.0
region.y_min, region.y_max = -1.0, 1.0
region.station_count = 7
assert len(addon.sample_path_components(settings)) == 10, "Disabled region must be ignored"

settings.station_plan_enabled = True
samples = addon.sample_path_components(settings)
inside = [sample for sample in samples if addon.point_in_station_region(sample[0], region)]
assert len(inside) == 7
assert all(sample[2] == first for sample in inside)
report = addon.station_plan_report(settings)
assert report["regions"] == [7] and report["requested"] == [7]

region.station_count = 3
report = addon.station_plan_report(settings)
assert report["regions"] == [3] and report["requested"] == [3]

# If a quota encloses a whole detail line, its established five-view minimum wins visibly.
region.x_min, region.x_max = -1.0, 11.0
region.y_min, region.y_max = -1.0, 1.0
region.station_count = 2
first["gs_detail_min_samples"] = 5
report = addon.station_plan_report(settings)
assert report["regions"] == [5] and report["requested"] == [2]

settings.station_plan_enabled = False
assert len(addon.sample_path_components(settings)) == 10, "Disabled mode must use existing detail minimum logic"
del first["gs_detail_min_samples"]
region.x_min, region.x_max = 2.0, 4.0
region.y_min, region.y_max = -1.0, 1.0
region.station_count = 7
settings.station_plan_enabled = True
settings.path_capture_mode = "LEGACY_PANORAMA_CUBE"
settings.path_station_array_mode = "SPHERICAL_SHELL_12"
settings.shell_radius_mode = "FIXED"
settings.shell_radius = 0.05
settings.shell_surface_clearance = 0.005
settings.shell_surface_detail_enabled = False
planned = addon.sample_path_components(settings)
cameras = addon.create_rig(scene, settings)
assert len(cameras) == len(planned) * 12
assert sum(camera.get("source_curve") == first.name for camera in cameras) >= 7 * 12
print("OPTIONAL_STATION_REGION_OK", report, flush=True)
addon.unregister()
