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

# The upper span represents a sealed model cavity. It overlaps the valid room
# in XY but has no stair or opening connecting it to the cursor's lower room.
cube("ValidFloor", (0.0, 0.0, -0.10), (8.0, 6.0, 0.20))
cube("SealedSlab", (0.0, 0.0, 3.00), (8.0, 6.0, 0.20))
cube("CavityRoof", (0.0, 0.0, 6.00), (8.0, 6.0, 0.20))
scene.cursor.location = (0.0, 0.0, 0.0)

settings = scene.gs_colmap_settings
settings.rig_mode = "PATH"
# Regression for the user's saved scene: the legacy panorama capture mode must
# still use the seeded reachable-space planner when multi-level planning is on.
settings.path_capture_mode = "LEGACY_PANORAMA_CUBE"
settings.scientific_origin_mode = "AUTO_GRID_PATH"
settings.multilevel_planning = True
settings.floorplan_space_mode = "REACHABLE"
settings.floorplan_seed_mode = "CURSOR"
settings.floorplan_probe_spacing = 0.40
settings.floorplan_curve_density = 2.5
settings.floorplan_wall_margin = 0.10
settings.floorplan_narrow_margin = 0.10
settings.floorplan_min_headroom = 1.0
settings.eye_height = 1.40
settings.multilevel_minimum_floor_cells = 4
settings.multilevel_large_room_area = 1000.0

result = addon.build_floorplan_path(scene, settings)
assert result is not None and result["objects"]
stats = json.loads(scene["gs_multilevel_path_stats"])
assert stats["detected_walkable_cell_count"] > stats["walkable_cell_count"], stats
assert stats["excluded_unreachable_cell_count"] > 0, stats
assert stats["floor_region_count"] == 1, stats
assert stats["reachable_seed_key"] is not None, stats

for obj in result["objects"]:
    for spline in obj.data.splines:
        # Automatic reachable paths are now true LOW/MID/HIGH layers.  All may
        # rise within the lower room, but none may enter the sealed upper floor.
        assert max(point.co.z for point in spline.points) < 3.0, (
            obj.name, [point.co.z for point in spline.points]
        )

print("REACHABLE_SPACE_ACCEPTANCE_OK", json.dumps(stats, sort_keys=True))
