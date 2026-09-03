import json
import sys
from pathlib import Path

import bpy


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

# Two furnished-scale rooms share one continuous floor and a genuine 1.60 m
# opening.  The wall pieces deliberately leave the doorway as empty geometry.
cube("ContinuousFloor", (0.0, 0.0, -0.10), (12.0, 6.0, 0.20))
cube("ContinuousCeiling", (0.0, 0.0, 3.10), (12.0, 6.0, 0.20))
cube("OuterNorth", (0.0, 3.05, 1.50), (12.2, 0.10, 3.0))
cube("OuterSouth", (0.0, -3.05, 1.50), (12.2, 0.10, 3.0))
cube("OuterWest", (-6.05, 0.0, 1.50), (0.10, 6.2, 3.0))
cube("OuterEast", (6.05, 0.0, 1.50), (0.10, 6.2, 3.0))
cube("DoorWallNorth", (0.0, 1.90, 1.50), (0.18, 2.20, 3.0))
cube("DoorWallSouth", (0.0, -1.90, 1.50), (0.18, 2.20, 3.0))
scene.cursor.location = (-3.0, 0.0, 0.0)

settings = scene.gs_colmap_settings
settings.live_update_cameras = False
settings.rig_mode = "PATH"
settings.path_capture_mode = "SCIENTIFIC_THREE_LAYER"
settings.scientific_origin_mode = "AUTO_GRID_PATH"
settings.multilevel_planning = True
settings.floorplan_space_mode = "REACHABLE"
settings.floorplan_seed_mode = "CURSOR"
settings.floorplan_probe_spacing = 0.40
settings.floorplan_curve_density = 2.50
settings.floorplan_wall_margin = 0.12
settings.floorplan_narrow_margin = 0.12
settings.floorplan_min_headroom = 1.0
settings.eye_height = 1.40
settings.scientific_camera_clearance = 0.25
settings.multilevel_minimum_floor_cells = 4
settings.multilevel_coverage_paths = 2
settings.multilevel_large_room_area = 1000.0
settings.fragment_stitching = False

for layer_count in (2, 3, 4):
    settings.scientific_layer_count = layer_count
    result = addon.build_floorplan_path(scene, settings)
    assert result is not None and result["objects"], layer_count
    stats = json.loads(scene["gs_multilevel_path_stats"])
    actual_layers = {int(obj["gs_path_layer_index"]) for obj in result["objects"]}
    assert actual_layers == set(range(layer_count)), (layer_count, actual_layers, stats)
    assert stats["actual_layer_count"] == layer_count, stats
    assert stats["room_region_count"] >= 2, stats
    assert stats["portal_count"] >= 1, stats
    assert stats["path_spatial_coverage_ratio"] >= 0.75, stats
    assert any(
        point.co.x > 1.0
        for obj in result["objects"]
        for spline in obj.data.splines
        for point in spline.points
    ), "the real doorway did not connect the cursor room to the second room"
    assert all(obj.get("gs_pre_layered_path") for obj in result["objects"])

print("DOOR_COVERAGE_LAYERS_ACCEPTANCE_OK", json.dumps(stats, sort_keys=True), flush=True)
