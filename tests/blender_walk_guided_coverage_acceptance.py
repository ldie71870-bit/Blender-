import json
import sys
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import blender_gs_colmap_exporter as addon


def cube(name, location, scale):
    bpy.ops.mesh.primitive_cube_add(location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = tuple(value * 0.5 for value in scale)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return obj


bpy.ops.wm.read_factory_settings(use_empty=True)
addon.register()
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0

# Two rooms have a genuine central door; a second XY-overlapping span above the
# ceiling is a sealed cavity.  The walk deliberately visits only room centres.
cube("Floor", (0.0, 0.0, -0.10), (12.0, 6.0, 0.20))
cube("Ceiling", (0.0, 0.0, 3.10), (12.0, 6.0, 0.20))
cube("OuterNorth", (0.0, 3.05, 1.50), (12.2, 0.10, 3.0))
cube("OuterSouth", (0.0, -3.05, 1.50), (12.2, 0.10, 3.0))
cube("OuterWest", (-6.05, 0.0, 1.50), (0.10, 6.2, 3.0))
cube("OuterEast", (6.05, 0.0, 1.50), (0.10, 6.2, 3.0))
cube("DividerNorth", (0.0, 1.90, 1.50), (0.18, 2.20, 3.0))
cube("DividerSouth", (0.0, -1.90, 1.50), (0.18, 2.20, 3.0))
cube("SealedCavityFloor", (0.0, 0.0, 6.00), (12.0, 6.0, 0.20))
cube("SealedCavityRoof", (0.0, 0.0, 9.00), (12.0, 6.0, 0.20))

settings = scene.gs_colmap_settings
settings.live_update_cameras = False
settings.floorplan_probe_spacing = 0.30
settings.floorplan_curve_density = 3.0
settings.floorplan_wall_margin = 0.10
settings.floorplan_narrow_margin = 0.10
settings.floorplan_min_headroom = 1.0
settings.eye_height = 1.40
settings.scientific_camera_clearance = 0.20
settings.multilevel_minimum_floor_cells = 4
settings.multilevel_planning = True
settings.fragment_stitching = False
settings.manual_walk_layer_count = "3"

walk = [Vector((-4.5 + 0.25 * index, 0.0, 1.55 + (0.02 if index % 2 else -0.02))) for index in range(37)]
addon.manual_walk_path.create_base_path(scene, walk)
summary = addon.manual_walk_path.regenerate_manual_layers(scene, settings, force_bvh=True)
assert summary is not None, summary
stats = json.loads(scene["gs_multilevel_path_stats"])
assert stats["walk_guided_coverage"], stats
assert stats["walk_seed_count"] >= 8, stats
assert stats["path_spatial_coverage_ratio"] >= 0.98, stats
assert stats["uncovered_cell_count"] == 0, stats
assert stats["actual_layer_count"] == 3, stats

collection = bpy.data.collections["GS_FloorPath_Auto"]
objects = [obj for obj in collection.objects if obj.get("gs_auto_reachable_layer")]
assert {obj["gs_path_layer_index"] for obj in objects} == {0, 1, 2}
assert any(
    point.co.x > 2.0
    for obj in objects for spline in obj.data.splines for point in spline.points
), "walk seeds did not cover the doorway-connected room"
assert all(
    point.co.z < 3.0
    for obj in objects for spline in obj.data.splines for point in spline.points
), "sealed upper cavity received paths"

print("WALK_GUIDED_COVERAGE_ACCEPTANCE_OK", json.dumps(stats, sort_keys=True), flush=True)
