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
    obj.scale = (scale[0] * 0.5, scale[1] * 0.5, scale[2] * 0.5)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return obj


bpy.ops.wm.read_factory_settings(use_empty=True)
addon.register()
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0

# Two slabs overlap in XY near the stair entrance; the upper slab is also the
# lower-storey ceiling. A separate roof closes the upper storey.
cube("Floor0", (2.0, 0.0, -0.10), (8.0, 4.0, 0.20))
cube("Floor1", (13.0, 0.0, 2.90), (6.0, 4.0, 0.20))
cube("Roof", (7.0, 0.0, 6.00), (20.0, 4.0, 0.20))

# Solid treads provide actual support throughout the connector. Each adjacent
# walkable top rises 0.30 m, below the configured maximum stair step.
for index in range(1, 11):
    top = 0.30 * index
    x = 3.7 + 0.60 * index
    cube(f"Step_{index:02d}", (x, 0.0, top * 0.5), (0.64, 2.0, top))

settings = scene.gs_colmap_settings
settings.rig_mode = "PATH"
settings.path_capture_mode = "SCIENTIFIC_THREE_LAYER"
settings.scientific_origin_mode = "AUTO_GRID_PATH"
settings.scientific_realization_mode = "SCIENTIFIC_POSE_SEQUENCE"
settings.multilevel_planning = True
settings.curve_smoothing = True
settings.fragment_stitching = True
settings.floorplan_probe_spacing = 0.30
settings.floorplan_curve_density = 3.0
settings.floorplan_wall_margin = 0.10
settings.floorplan_narrow_margin = 0.10
settings.floorplan_min_headroom = 1.0
settings.eye_height = 1.40
settings.multilevel_floor_tolerance = 0.08
settings.multilevel_maximum_step = 0.35
settings.multilevel_minimum_floor_cells = 4
settings.multilevel_large_room_area = 1000.0
settings.scientific_show_debug = True

result = addon.build_floorplan_path(scene, settings)
assert result is not None, "3D planner produced no path"
stats = json.loads(scene["gs_multilevel_path_stats"])
plan = result["planner_result"]
assert stats["floor_region_count"] >= 2, stats
assert stats["stair_connector_count"] >= 1, stats
assert 1 <= stats["final_spline_count"] <= stats["raw_fragment_count"], stats
assert any(len(fragment.region_ids) >= 2 and fragment.connector_id for fragment in plan.final_fragments)
assert any(obj.get("gs_connector_id") for obj in result["objects"]), "stair was not stitched into a final spline"
assert all(obj.type == "CURVE" for obj in result["objects"])
assert not any(obj.type == "CAMERA" for obj in scene.objects), "path planning must not create pose cameras"

debug = bpy.data.collections.get("GS_SCIENTIFIC_PATH_DEBUG")
categories = {obj.get("gs_path_debug_category") for obj in debug.objects}
assert {
    "ALL_WALKABLE", "REACHABLE", "SEED", "FLOOR_REGION",
    "RAW_FRAGMENT", "FINAL_SPLINE", "STAIR_CONNECTION",
} <= categories, categories
assert all(obj.hide_render for obj in debug.objects if obj.get("gs_path_planner_debug"))

print("MULTILEVEL_SMOKE_OK", json.dumps(stats, sort_keys=True))
