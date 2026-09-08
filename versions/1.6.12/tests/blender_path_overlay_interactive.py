"""Interactive viewport smoke test; run without --background so GPU drawing executes."""
import bpy
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "path_overlay_ui_test_addon", ROOT / "__init__.py",
    submodule_search_locations=[str(ROOT)],
)
addon = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = addon
spec.loader.exec_module(addon)
addon.register()

scene = bpy.context.scene
settings = scene.gs_colmap_settings
paths = bpy.data.collections.new("GPU Overlay Paths")
scene.collection.children.link(paths)
settings.path_collection = paths
curve_data = bpy.data.curves.new("GPU Overlay Curve Data", "CURVE")
curve_data.dimensions = "3D"
spline = curve_data.splines.new("POLY")
spline.points.add(1)
spline.points[0].co = (-1.0, 0.0, 1.5, 1.0)
spline.points[1].co = (1.0, 0.0, 1.5, 1.0)
curve = bpy.data.objects.new("GPU Overlay Curve", curve_data)
paths.objects.link(curve)
for index in range(12):
    data = bpy.data.cameras.new(f"GPU Camera {index} Data")
    camera = bpy.data.objects.new(f"GPU Camera {index}", data)
    scene.collection.objects.link(camera)
    camera.location = (index / 6.0 - 1.0, 0.0, 0.5)
    camera["source_curve"] = curve.name
    camera["station_index"] = index
    camera["shell_camera_index"] = index
curve.select_set(True)
bpy.context.view_layer.objects.active = curve
bpy.ops.gs_colmap.toggle_path_inspection()
settings.station_plan_enabled = True
region = settings.station_regions.add()
region.name = "GPU Quota"
region.x_min, region.x_max = -0.5, 0.5
region.y_min, region.y_max = -0.5, 0.5
region.station_count = 7
area = next(area for area in bpy.context.window.screen.areas if area.type == "VIEW_3D")
window_region = next(region for region in area.regions if region.type == "WINDOW")
with bpy.context.temp_override(area=area, region=window_region, space_data=area.spaces.active):
    assert bpy.ops.gs_colmap.toggle_station_plan_view() == {"FINISHED"}
assert settings.station_plan_view_active
validation = ROOT.parent / "validation_1.6.6"
validation.mkdir(exist_ok=True)
plan_image = bpy.data.images[addon.STATION_PLAN_IMAGE_NAME]
plan_image.filepath_raw = str(validation / "cad_plan_smoke.png")
plan_image.file_format = "PNG"
plan_image.save()
addon.tag_view3d_redraw()


def finish():
    image_area = next(area for area in bpy.context.window.screen.areas if area.type == "IMAGE_EDITOR")
    image_region = next(region for region in image_area.regions if region.type == "WINDOW")
    with bpy.context.temp_override(area=image_area, region=image_region, space_data=image_area.spaces.active):
        assert bpy.ops.gs_colmap.toggle_station_plan_view() == {"FINISHED"}
    assert image_area.type == "VIEW_3D" and not settings.station_plan_view_active
    print("PATH_AND_STATION_OVERLAY_INTERACTIVE_OK", flush=True)
    bpy.ops.wm.quit_blender()
    return None


bpy.app.timers.register(finish, first_interval=2.0)
