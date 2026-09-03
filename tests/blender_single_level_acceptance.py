import json
import sys
from pathlib import Path

import bpy
from mathutils import Matrix


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


def sample(index, matrix):
    return addon.pose_sequence.PoseSample(
        logical_frame_id=index, image_id=index, segment_id=1, order_in_segment=index,
        source_curve_name="acceptance", source_curve_index=0,
        source_path_distance=float(index - 1), layer_index=1, layer_type="Middle",
        sample_type="BASE", matrix_world=matrix, focal_length_mm=35.0,
        sensor_width_mm=36.0, sensor_height_mm=24.0, sensor_fit="AUTO",
        shift_x=0.0, shift_y=0.0, resolution_x=640, resolution_y=480,
        horizontal_fov=1.0, vertical_fov=0.8, coverage_score=1.0,
        overlap_score=0.7, near_field_class="REGULAR", candidate_id=f"pose_{index}",
        provider_type="PATH", region_id="Floor_0",
    )


bpy.ops.wm.read_factory_settings(use_empty=True)
addon.register()
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0
cube("Floor", (0.0, 0.0, -0.1), (8.0, 6.0, 0.2))
cube("Ceiling", (0.0, 0.0, 3.0), (8.0, 6.0, 0.2))

settings = scene.gs_colmap_settings
settings.rig_mode = "PATH"
settings.path_capture_mode = "SCIENTIFIC_THREE_LAYER"
settings.scientific_origin_mode = "AUTO_GRID_PATH"
settings.scientific_realization_mode = "SCIENTIFIC_POSE_SEQUENCE"
settings.multilevel_planning = True
settings.floorplan_probe_spacing = 0.40
settings.floorplan_curve_density = 2.5
settings.floorplan_wall_margin = 0.10
settings.floorplan_narrow_margin = 0.10
settings.eye_height = 1.40
settings.multilevel_minimum_floor_cells = 4
settings.multilevel_large_room_area = 12.0

path = addon.build_floorplan_path(scene, settings)
assert path is not None and path["objects"]
stats = json.loads(scene["gs_multilevel_path_stats"])
assert stats["floor_region_count"] == 1, stats
assert stats["connector_count"] == 0, stats
assert all(max(point.co.z for point in obj.data.splines[0].points) -
           min(point.co.z for point in obj.data.splines[0].points) < 1e-4
           for obj in path["objects"]), "single-floor paths must stay level"

# PoseSequence remains the authority while one real camera receives optional
# preview location/quaternion keyframes.
collection = bpy.data.collections.new("GS_COLMAP_Cameras")
scene.collection.children.link(collection)
sequence = addon.pose_sequence.PoseSequence(
    source_scene_frame=1, shared_intrinsics={}, sequence_intrinsics_hash="x",
    scene_hash="scene", planning_hash="plan",
    segments=[{"segment_id": 1}],
    frames=[
        sample(1, Matrix.Translation((0.0, 0.0, 1.4))),
        sample(2, Matrix.Translation((1.0, 0.0, 1.4))),
        sample(3, Matrix.Translation((2.0, 0.5, 1.4))),
    ],
)
capture = addon._capture_camera(collection, settings, sequence)
addon.pose_sequence.create_preview_keyframes(scene, capture, sequence)
assert capture.name == "GS_CAPTURE_CAMERA"
assert len([obj for obj in scene.objects if obj.type == "CAMERA"]) == 1
assert capture.rotation_mode == "QUATERNION"
assert sequence.preview_keyframes_created
action = capture.animation_data.action
paths = set()
for layer in action.layers:
    for strip in layer.strips:
        for channelbag in strip.channelbags:
            paths.update(curve.data_path for curve in channelbag.fcurves)
assert {"location", "rotation_quaternion"} <= paths, paths

print("SINGLE_LEVEL_ACCEPTANCE_OK", json.dumps(stats, sort_keys=True))
