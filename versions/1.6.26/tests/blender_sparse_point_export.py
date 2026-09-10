"""Verify fast sparse point sampling and no-rerender COLMAP export in Blender.

Run with Blender --background --factory-startup --python-exit-code 1 --python
this_file.py.
"""
import importlib.util
import tempfile
import sys
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "sparse_point_export_addon", ROOT / "__init__.py",
    submodule_search_locations=[str(ROOT)],
)
addon = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = addon
spec.loader.exec_module(addon)
addon.register()

scene = bpy.context.scene
settings = scene.gs_colmap_settings
settings.point_samples_per_view = 16
settings.point_dedup_size = 0.02
settings.ray_distance = 50.0


def cube(name, location, scale):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = scale
    return obj


cube("SparseFixtureFloor", (0.0, 0.0, -0.2), (8.0, 8.0, 0.2))
cube("SparseFixtureWall", (0.0, 3.0, 2.0), (8.0, 0.2, 3.0))


def camera(name, location, target):
    data = bpy.data.cameras.new(name)
    data.type = "PERSP"
    data.lens = 28.0
    obj = bpy.data.objects.new(name, data)
    scene.collection.objects.link(obj)
    obj.location = location
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()
    return obj


cameras = [
    camera("SparseFixtureCameraA", (0.0, -2.0, 1.2), (0.0, 3.0, 1.5)),
    camera("SparseFixtureCameraB", (1.0, -1.5, 1.6), (0.0, 3.0, 1.6)),
    camera("SparseFixtureCameraC", (-1.0, -1.5, 1.4), (0.0, 3.0, 1.4)),
]
bpy.context.view_layer.update()

constructed = []
original_query = addon.capture_collision.CollisionQuery


class CountingQuery(original_query):
    def __init__(self, *args, **kwargs):
        constructed.append(1)
        super().__init__(*args, **kwargs)


addon.capture_collision.CollisionQuery = CountingQuery
progress = []
try:
    points = addon.sample_sparse_points(
        scene, cameras, settings, lambda current, total: progress.append((current, total)),
    )
finally:
    addon.capture_collision.CollisionQuery = original_query

assert len(constructed) == 1, constructed
assert len(points) > 0
assert progress == [(0, 3), (1, 3), (2, 3), (3, 3)], progress
assert scene.as_pointer() not in addon.capture_collision.SCOPES

with tempfile.TemporaryDirectory(prefix="gs_sparse_export_test_") as temporary:
    settings.output_dir = temporary
    names = ["frame_0001.png", "frame_0002.png", "frame_0003.png"]
    addon.write_colmap(scene, settings, cameras, names, points)
    root = Path(temporary) / "sparse" / "0"
    assert (root / "cameras.txt").is_file()
    assert (root / "images.txt").is_file()
    assert (root / "points3D.txt").is_file()
    assert (Path(temporary) / "transforms.json").is_file()
    images = (root / "images.txt").read_text(encoding="utf8")
    point_lines = (root / "points3D.txt").read_text(encoding="utf8").splitlines()
    assert sum(1 for line in images.splitlines() if line and not line.startswith("#")) == 3
    assert len(point_lines) > 1

addon.unregister()
print("SPARSE_POINT_EXPORT_OK", len(points), flush=True)
