import ast
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector


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


def valid_objects(layer=None):
    collection = bpy.data.collections.get(addon.manual_walk_path.VALID_COLLECTION)
    result = [obj for obj in collection.objects if obj.get("gs_manual_valid_segment")]
    if layer:
        result = [obj for obj in result if obj.get("gs_manual_layer_name") == layer]
    return sorted(result, key=lambda obj: obj.name)


def z_range(obj):
    points = [obj.matrix_world @ point.co.xyz for spline in obj.data.splines for point in spline.points]
    return max(point.z for point in points) - min(point.z for point in points)


def pose_sample(index, position):
    return addon.pose_sequence.PoseSample(
        logical_frame_id=index, image_id=index, segment_id=1, order_in_segment=index,
        source_curve_name="manual", source_curve_index=0, source_path_distance=float(index),
        layer_index=1, layer_type="Middle", sample_type="BASE",
        matrix_world=Matrix.Translation(position), focal_length_mm=35.0,
        sensor_width_mm=36.0, sensor_height_mm=24.0, sensor_fit="AUTO",
        shift_x=0.0, shift_y=0.0, resolution_x=640, resolution_y=480,
        horizontal_fov=1.0, vertical_fov=0.8, coverage_score=1.0,
        overlap_score=0.7, near_field_class="REGULAR", candidate_id=f"manual_{index}",
        provider_type="PATH", region_id="manual",
    )


bpy.ops.wm.read_factory_settings(use_empty=True)
addon.register()
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0
settings = scene.gs_colmap_settings
settings.live_update_cameras = False
settings.manual_walk_layer_count = "3"
settings.manual_walk_layer_spacing = 0.35
settings.manual_walk_collision_cut = False

# Raw walk processing retains a real one-floor -> stair -> two-floor transition.
raw = []
for index in range(80):
    x = index * 0.10
    z = 1.50 if x < 2.0 else 1.50 + min(2.0, (x - 2.0) * 0.5)
    raw.append(Vector((x, 0.0, z)))
processed = addon.manual_walk_path.process_raw_samples(raw, 1.0)
assert len(processed) > 20
assert max(point.z for point in processed) - min(point.z for point in processed) > 1.8
addon.manual_walk_path.create_base_path(scene, processed)
summary = addon.manual_walk_path.regenerate_manual_layers(scene, settings)
assert summary["valid_segments"] == 3, summary
assert all(z_range(obj) > 1.8 for obj in valid_objects()), "layer offsets flattened the stairs"

# Ordinary room with independent upper/lower obstacles: each collision zone is
# merged and cut once, while the middle layer stays whole.
straight = [Vector((index * 0.10, 0.0, 1.50)) for index in range(101)]
addon.manual_walk_path.create_base_path(scene, straight)
cube("Floor", (5.0, 0.0, -0.10), (14.0, 4.0, 0.20))
cube("Ceiling", (5.0, 0.0, 3.10), (14.0, 4.0, 0.20))
cube("UpperObstacle", (4.0, 0.0, 1.85), (1.0, 1.0, 0.20))
cube("LowerObstacle", (7.0, 0.0, 1.15), (1.0, 1.0, 0.20))
settings.manual_walk_collision_cut = True
settings.manual_walk_safety_radius = 0.20
summary = addon.manual_walk_path.regenerate_manual_layers(scene, settings, force_bvh=True)
by_name = {item["name"]: item for item in summary["layers"]}
assert by_name["Upper"]["valid_segments"] == 2, summary
assert by_name["Middle"]["valid_segments"] == 1, summary
assert by_name["Lower"]["valid_segments"] == 2, summary
assert summary["invalid_intervals"] == 2, summary
debug = bpy.data.collections.get(addon.manual_walk_path.DEBUG_COLLECTION)
assert len([obj for obj in debug.objects if obj.get("gs_manual_collision_debug")]) == 2

# Offset edits regenerate from BasePath. Raising first clears the low obstacle;
# raising again reaches a local ceiling beam and splits at the new interval.
cube("LocalCeilingBeam", (2.0, 0.0, 2.50), (1.0, 1.0, 0.20))
settings.manual_walk_layer_1_offset = 0.70
restored_count = len(valid_objects("Upper"))
settings.manual_walk_layer_1_offset = 0.90
raised_count = len(valid_objects("Upper"))
assert restored_count == 1, restored_count
assert raised_count == 2, raised_count
assert bpy.data.objects.get(addon.manual_walk_path.BASE_OBJECT) is not None

# Pre-layered valid curves enter the scientific provider at their exact XYZ;
# they must not be expanded into another 2/3/4 height set.
components = addon.path_components(settings)
component_counts = [2] * len(components)
cache = addon.scientific_planner.make_cache(scene, settings)
groups, stations, _unsafe = addon.scientific_planner._make_curve_candidate_groups(
    cache, components, component_counts, cast_rays=False
)
assert len(groups) == sum(len(items) for items in stations), (len(groups), [len(items) for items in stations])
for group in groups:
    candidate = group[0]
    source = components[candidate.component_index]["object"]
    source_points = components[candidate.component_index]["points"]
    assert min(abs(candidate.position.z - point.z) for point in source_points) < 1e-5
    assert candidate.layer_name == source.get("gs_manual_layer_name")

# Existing PoseSequence realization remains one physical Blender camera.
camera_collection = bpy.data.collections.new("GS_COLMAP_Cameras")
scene.collection.children.link(camera_collection)
sequence = addon.pose_sequence.PoseSequence(
    source_scene_frame=1, shared_intrinsics={}, sequence_intrinsics_hash="manual",
    scene_hash="manual", planning_hash="manual", segments=[{"segment_id": 1}],
    frames=[pose_sample(1, (0.0, 0.0, 1.5)), pose_sample(2, (1.0, 0.0, 1.5))],
)
capture = addon._capture_camera(camera_collection, settings, sequence)
assert capture.name == "GS_CAPTURE_CAMERA"
assert len([obj for obj in scene.objects if obj.type == "CAMERA"]) == 1

print("MANUAL_WALK_ACCEPTANCE_OK", ast.literal_eval(scene["gs_manual_walk_summary"]), flush=True)
