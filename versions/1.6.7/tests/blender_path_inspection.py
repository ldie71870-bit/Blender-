"""Acceptance checks for the zero-object single-path camera overlay."""
import bpy
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "path_inspection_test_addon", ROOT / "__init__.py",
    submodule_search_locations=[str(ROOT)],
)
addon = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = addon
spec.loader.exec_module(addon)
addon.register()

scene = bpy.context.scene
for obj in list(scene.objects):
    bpy.data.objects.remove(obj, do_unlink=True)

paths = bpy.data.collections.new("Inspection Paths")
scene.collection.children.link(paths)
settings = scene.gs_colmap_settings
settings.path_collection = paths


def curve(name, z):
    data = bpy.data.curves.new(name + " Data", "CURVE")
    data.dimensions = "3D"
    spline = data.splines.new("POLY")
    spline.points.add(1)
    spline.points[0].co = (0.0, 0.0, z, 1.0)
    spline.points[1].co = (1.0, 0.0, z, 1.0)
    obj = bpy.data.objects.new(name, data)
    paths.objects.link(obj)
    return obj


target = curve("Inspect Me", 1.0)
other = curve("Hide Me", 2.0)
bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0.0, 2.0, 0.5))
furniture = bpy.context.object
furniture.name = "Scene Furniture"

cameras = []
for index, source in enumerate((target.name, target.name, other.name)):
    data = bpy.data.cameras.new(f"Camera {index} Data")
    obj = bpy.data.objects.new(f"Camera {index}", data)
    scene.collection.objects.link(obj)
    obj.location = (index, 0.0, 1.0)
    obj["source_curve"] = source
    obj["station_index"] = index
    obj["shell_camera_index"] = index
    cameras.append(obj)
cameras[2].hide_set(True)

# A legacy proxy must be removed, while no replacement object is created.
legacy_mesh = bpy.data.meshes.new(addon.CAMERA_VISUAL_MESH)
legacy = bpy.data.objects.new("Legacy Pyramid", legacy_mesh)
scene.collection.objects.link(legacy)
legacy["gs_camera_mesh_visual"] = True
legacy.parent = cameras[0]
cameras[0]["gs_camera_mesh_visual_name"] = legacy.name
cameras[0]["gs_camera_original_display_size"] = 0.5
cameras[0].data.display_size = 0.01

for obj in bpy.context.selected_objects:
    obj.select_set(False)
target.select_set(True)
bpy.context.view_layer.objects.active = target
bpy.context.view_layer.update()
before_count = len(bpy.data.objects)

result = bpy.ops.gs_colmap.toggle_path_inspection()
assert result == {"FINISHED"}
assert settings.path_inspection_enabled
assert settings.path_inspection_curve == target
assert not target.hide_get() and target.show_in_front
assert other.hide_get(), "Other path should be hidden"
assert not furniture.hide_get(), "Scene geometry must remain visible"
assert all(camera.hide_get() for camera in cameras), "Native camera wireframes must be hidden"
assert len(addon.path_inspection_cameras(scene, target)) == 2
points, lines = addon._path_inspection_geometry(scene, settings)
assert len(points) == 2 and len(lines) == 4
assert bpy.data.objects.get("Legacy Pyramid") is None
assert len(bpy.data.objects) == before_count - 1, "Inspection must not create proxy objects"

result = bpy.ops.gs_colmap.toggle_path_inspection()
assert result == {"FINISHED"}
assert not settings.path_inspection_enabled and settings.path_inspection_curve is None
assert not target.hide_get() and not target.show_in_front
assert not other.hide_get()
assert not cameras[0].hide_get() and not cameras[1].hide_get()
assert cameras[2].hide_get(), "Pre-existing camera visibility must be restored"
assert not furniture.hide_get()
assert cameras[0].data.display_size == 0.5

print("PATH_INSPECTION_OK", len(points), len(lines), flush=True)
addon.unregister()
