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
spline.points[0].co = (-1.0, 0.0, 0.0, 1.0)
spline.points[1].co = (1.0, 0.0, 0.0, 1.0)
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
addon.tag_view3d_redraw()


def finish():
    print("PATH_OVERLAY_INTERACTIVE_OK", flush=True)
    bpy.ops.wm.quit_blender()
    return None


bpy.app.timers.register(finish, first_interval=2.0)
