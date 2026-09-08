"""Authoritative pose-sequence data model for scientific dataset capture."""

from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

import bpy
from mathutils import Matrix, Quaternion, Vector


SCHEMA_VERSION = 1
MODE = "SCIENTIFIC_POSE_SEQUENCE"
SEQUENCE_FILENAME = "camera_sequence.json"
CAPTURE_CAMERA_NAME = "GS_CAPTURE_CAMERA"
SCENE_SEQUENCE_KEY = "gs_camera_sequence_json"
PREVIEW_MARKER_PREFIX = "GS_SEGMENT_"
STATUS_VALUES = {"PENDING", "RENDERING", "COMPLETE", "FAILED", "SKIPPED"}


def matrix_to_list(matrix):
    return [[float(value) for value in row] for row in matrix]


def matrix_from_list(values):
    return Matrix(tuple(tuple(float(value) for value in row) for row in values))


def _json_hash(payload):
    raw = json.dumps(payload, ensure_ascii=True, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(raw.encode("utf-8")).hexdigest()


def _quaternion_list(matrix):
    quat = matrix.to_quaternion().normalized()
    return [float(quat.w), float(quat.x), float(quat.y), float(quat.z)]


def build_output_paths(logical_frame_id, settings, patch_index=0):
    extension = ".png" if settings.image_format == "PNG" else ".jpg"
    if patch_index:
        stem = f"{settings.image_prefix}_patch_{patch_index:06d}"
    else:
        stem = f"{settings.image_prefix}_{logical_frame_id:06d}"
    depth_extension = ".exr" if getattr(settings, "depth_format", "EXR") == "EXR" else ".png"
    physical = getattr(settings, "object_split_mode", "VIRTUAL_SPLIT") == "PHYSICAL_FILES"
    object_depth = bool(getattr(settings, "export_object_depth", False))
    object_normal = bool(getattr(settings, "export_object_normal", False))
    object_mask = bool(getattr(settings, "export_object_mask", False))
    virtual_object_data = not physical and (object_depth or object_normal or object_mask)
    return {
        "stem": stem,
        "rgb": f"images/{stem}{extension}",
        "depth": f"depth/{stem}{depth_extension}" if getattr(settings, "export_depth", False) or (not physical and object_depth) else "",
        "normal": f"normal/{stem}.png" if getattr(settings, "export_normal", False) or (not physical and object_normal) else "",
        "id": f"id/{stem}.png" if getattr(settings, "export_id", False) or virtual_object_data else "",
        "material_id": f"material_id/{stem}.png" if getattr(settings, "export_material_id", False) else "",
        "object_depth": f"objects/<item>/depth/{stem}{depth_extension}" if physical and object_depth else "",
        "object_normal": f"objects/<item>/normal/{stem}.png" if physical and object_normal else "",
        "mask": f"objects/<item>/mask/{stem}.png" if physical and object_mask else "",
    }


def shared_intrinsics(scene, settings, camera_data):
    return {
        "focal_length_mm": float(camera_data.lens),
        "sensor_width_mm": float(camera_data.sensor_width),
        "sensor_height_mm": float(camera_data.sensor_height),
        "sensor_fit": str(camera_data.sensor_fit),
        "shift_x": float(camera_data.shift_x),
        "shift_y": float(camera_data.shift_y),
        "resolution_x": int(settings.resolution_x),
        "resolution_y": int(settings.resolution_y),
        "resolution_percentage": 100,
        "pixel_aspect_x": float(scene.render.pixel_aspect_x),
        "pixel_aspect_y": float(scene.render.pixel_aspect_y),
        "camera_type": str(camera_data.type),
    }


def intrinsics_hash(intrinsics):
    return _json_hash(intrinsics)


def scene_signature(scene, settings, source_frame):
    objects = []
    for obj in sorted(scene.objects, key=lambda item: item.name_full):
        if obj.get("gs_camera_mesh_visual") or obj.get("gs_scientific_debug"):
            continue
        if obj.type == "CAMERA":
            continue
        objects.append({
            "name": obj.name_full,
            "type": obj.type,
            "data": getattr(getattr(obj, "data", None), "name_full", ""),
            "matrix": matrix_to_list(obj.matrix_world),
            "hide_render": bool(obj.hide_render),
        })
    payload = {
        "scene": scene.name_full,
        "source_frame": int(source_frame),
        "objects": objects,
        "render_engine": str(settings.render_engine),
        "origin_mode": str(getattr(settings, "scientific_origin_mode", "MANUAL_CURVE")),
        "budget_mode": str(getattr(settings, "scientific_budget_mode", "LEGACY_PATH_BUDGET")),
        "fixed_budget": int(getattr(settings, "scientific_fixed_budget", 0)),
        "focal_length": float(settings.focal_length),
    }
    return _json_hash(payload)


def planning_settings_signature(settings):
    keys = (
        "path_count_mode", "camera_count", "path_camera_density", "max_path_cameras",
        "path_samples_per_segment", "scientific_origin_mode", "scientific_budget_mode",
        "scientific_fixed_budget", "scientific_minimum_budget", "scientific_maximum_budget",
        "scientific_layer_count", "scientific_minimum_overlap", "scientific_target_overlap",
        "scientific_minimum_step", "scientific_maximum_step", "scientific_camera_clearance",
        "scientific_view_budget_multiplier", "scientific_minimum_observations",
        "scientific_preferred_observations", "scientific_ray_quality", "scientific_auto_coverage",
        "scientific_auto_floor_ceiling", "scientific_maximum_heading_change",
        "scientific_maximum_incidence_angle", "scientific_global_reachable_coverage",
        "scientific_coverage_driven", "scientific_post_clipping_recast",
        "near_field_protection", "sequence_source_scene_frame",
    )
    return _json_hash({key: getattr(settings, key, None) for key in keys})


def output_settings_signature(settings):
    keys = (
        "image_prefix", "image_format", "resolution_x", "resolution_y", "render_engine",
        "cycles_samples", "cycles_denoise", "cycles_device", "color_look", "color_exposure",
        "transparent_background", "render_rgb", "export_depth", "depth_format", "export_id",
        "export_object_depth", "export_normal", "export_object_normal", "export_object_mask",
        "export_material_id", "object_split_mode", "object_group_mode",
    )
    return _json_hash({key: getattr(settings, key, None) for key in keys})


@dataclass
class PoseSample:
    logical_frame_id: int
    image_id: int
    segment_id: int
    order_in_segment: int
    source_curve_name: str
    source_curve_index: int
    source_path_distance: float
    layer_index: int
    layer_type: str
    sample_type: str
    matrix_world: Matrix
    focal_length_mm: float
    sensor_width_mm: float
    sensor_height_mm: float
    sensor_fit: str
    shift_x: float
    shift_y: float
    resolution_x: int
    resolution_y: int
    horizontal_fov: float
    vertical_fov: float
    coverage_score: float
    overlap_score: float
    near_field_class: str
    is_polar_patch: bool = False
    is_bridge: bool = False
    is_coverage_patch: bool = False
    render_enabled: bool = True
    rgb_path: str = ""
    depth_path: str = ""
    normal_path: str = ""
    id_path: str = ""
    material_id_path: str = ""
    object_depth_path: str = ""
    object_normal_path: str = ""
    mask_path: str = ""
    render_status: str = "PENDING"
    candidate_id: str = ""
    provider_type: str = ""
    region_id: str = ""
    verification_position_error: float = 0.0
    verification_rotation_error: float = 0.0
    error: str = ""

    def to_dict(self):
        location = self.matrix_world.translation
        return {
            "logical_frame_id": int(self.logical_frame_id),
            "image_id": int(self.image_id),
            "segment_id": int(self.segment_id),
            "order_in_segment": int(self.order_in_segment),
            "source_curve_name": self.source_curve_name,
            "source_curve_index": int(self.source_curve_index),
            "source_path_distance": float(self.source_path_distance),
            "layer_index": int(self.layer_index),
            "layer_type": self.layer_type,
            "sample_type": self.sample_type,
            "matrix_world_blender": matrix_to_list(self.matrix_world),
            "location": [float(location.x), float(location.y), float(location.z)],
            "rotation_quaternion": _quaternion_list(self.matrix_world),
            "focal_length_mm": float(self.focal_length_mm),
            "sensor_width_mm": float(self.sensor_width_mm),
            "sensor_height_mm": float(self.sensor_height_mm),
            "sensor_fit": self.sensor_fit,
            "shift_x": float(self.shift_x),
            "shift_y": float(self.shift_y),
            "resolution_x": int(self.resolution_x),
            "resolution_y": int(self.resolution_y),
            "horizontal_fov": float(self.horizontal_fov),
            "vertical_fov": float(self.vertical_fov),
            "coverage_score": float(self.coverage_score),
            "overlap_score": float(self.overlap_score),
            "near_field_class": self.near_field_class,
            "is_polar_patch": bool(self.is_polar_patch),
            "is_bridge": bool(self.is_bridge),
            "is_coverage_patch": bool(self.is_coverage_patch),
            "render_enabled": bool(self.render_enabled),
            "rgb_path": self.rgb_path,
            "depth_path": self.depth_path,
            "normal_path": self.normal_path,
            "id_path": self.id_path,
            "material_id_path": self.material_id_path,
            "object_depth_path": self.object_depth_path,
            "object_normal_path": self.object_normal_path,
            "mask_path": self.mask_path,
            "file_path": self.rgb_path,
            "render_status": self.render_status,
            "candidate_id": self.candidate_id,
            "provider_type": self.provider_type,
            "region_id": self.region_id,
            "verification_position_error": float(self.verification_position_error),
            "verification_rotation_error": float(self.verification_rotation_error),
            "error": self.error,
        }

    @classmethod
    def from_dict(cls, data):
        status = str(data.get("render_status", "PENDING"))
        if status not in STATUS_VALUES:
            status = "PENDING"
        return cls(
            logical_frame_id=int(data["logical_frame_id"]),
            image_id=int(data["image_id"]),
            segment_id=int(data.get("segment_id", 1)),
            order_in_segment=int(data.get("order_in_segment", 1)),
            source_curve_name=str(data.get("source_curve_name", data.get("source_curve", ""))),
            source_curve_index=int(data.get("source_curve_index", 0)),
            source_path_distance=float(data.get("source_path_distance", 0.0)),
            layer_index=int(data.get("layer_index", 0)),
            layer_type=str(data.get("layer_type", "Middle")),
            sample_type=str(data.get("sample_type", "BASE")),
            matrix_world=matrix_from_list(data["matrix_world_blender"]),
            focal_length_mm=float(data.get("focal_length_mm", 35.0)),
            sensor_width_mm=float(data.get("sensor_width_mm", 36.0)),
            sensor_height_mm=float(data.get("sensor_height_mm", 32.0)),
            sensor_fit=str(data.get("sensor_fit", "AUTO")),
            shift_x=float(data.get("shift_x", 0.0)),
            shift_y=float(data.get("shift_y", 0.0)),
            resolution_x=int(data.get("resolution_x", 1536)),
            resolution_y=int(data.get("resolution_y", 1536)),
            horizontal_fov=float(data.get("horizontal_fov", 0.0)),
            vertical_fov=float(data.get("vertical_fov", 0.0)),
            coverage_score=float(data.get("coverage_score", 0.0)),
            overlap_score=float(data.get("overlap_score", 0.0)),
            near_field_class=str(data.get("near_field_class", "REGULAR")),
            is_polar_patch=bool(data.get("is_polar_patch", False)),
            is_bridge=bool(data.get("is_bridge", False)),
            is_coverage_patch=bool(data.get("is_coverage_patch", False)),
            render_enabled=bool(data.get("render_enabled", True)),
            rgb_path=str(data.get("rgb_path", data.get("file_path", ""))),
            depth_path=str(data.get("depth_path", "")),
            normal_path=str(data.get("normal_path", "")),
            id_path=str(data.get("id_path", "")),
            material_id_path=str(data.get("material_id_path", "")),
            object_depth_path=str(data.get("object_depth_path", "")),
            object_normal_path=str(data.get("object_normal_path", "")),
            mask_path=str(data.get("mask_path", "")),
            render_status=status,
            candidate_id=str(data.get("candidate_id", "")),
            provider_type=str(data.get("provider_type", "")),
            region_id=str(data.get("region_id", "")),
            verification_position_error=float(data.get("verification_position_error", 0.0)),
            verification_rotation_error=float(data.get("verification_rotation_error", 0.0)),
            error=str(data.get("error", "")),
        )


@dataclass
class PoseSequence:
    source_scene_frame: int
    shared_intrinsics: dict
    sequence_intrinsics_hash: str
    scene_hash: str
    planning_hash: str
    planning_settings_hash: str = ""
    output_settings_hash: str = ""
    segments: list = field(default_factory=list)
    frames: list = field(default_factory=list)
    mode: str = MODE
    schema_version: int = SCHEMA_VERSION
    ordering_method: str = "OVERLAP_GRAPH"
    ordering_frozen: bool = False
    preview_keyframes_created: bool = False
    motion_blur_disabled: bool = True
    scene_animation_frozen: bool = True
    max_position_verification_error: float = 0.0
    max_rotation_verification_error: float = 0.0

    def to_dict(self):
        return {
            "schema_version": int(self.schema_version),
            "mode": self.mode,
            "source_scene_frame": int(self.source_scene_frame),
            "shared_intrinsics": self.shared_intrinsics,
            "sequence_intrinsics_hash": self.sequence_intrinsics_hash,
            "scene_hash": self.scene_hash,
            "planning_hash": self.planning_hash,
            "planning_settings_hash": self.planning_settings_hash,
            "output_settings_hash": self.output_settings_hash,
            "ordering_method": self.ordering_method,
            "ordering_frozen": bool(self.ordering_frozen),
            "preview_keyframes_created": bool(self.preview_keyframes_created),
            "motion_blur_disabled": bool(self.motion_blur_disabled),
            "scene_animation_frozen": bool(self.scene_animation_frozen),
            "max_position_verification_error": float(self.max_position_verification_error),
            "max_rotation_verification_error": float(self.max_rotation_verification_error),
            "segments": self.segments,
            "frames": [sample.to_dict() for sample in self.frames],
        }

    @classmethod
    def from_dict(cls, data):
        if int(data.get("schema_version", 0)) != SCHEMA_VERSION or data.get("mode") != MODE:
            raise ValueError("Unsupported camera sequence schema or mode")
        return cls(
            source_scene_frame=int(data.get("source_scene_frame", 1)),
            shared_intrinsics=dict(data.get("shared_intrinsics", {})),
            sequence_intrinsics_hash=str(data.get("sequence_intrinsics_hash", "")),
            scene_hash=str(data.get("scene_hash", "")),
            planning_hash=str(data.get("planning_hash", "")),
            planning_settings_hash=str(data.get("planning_settings_hash", "")),
            output_settings_hash=str(data.get("output_settings_hash", "")),
            segments=list(data.get("segments", [])),
            frames=[PoseSample.from_dict(item) for item in data.get("frames", [])],
            ordering_method=str(data.get("ordering_method", "OVERLAP_GRAPH")),
            ordering_frozen=bool(data.get("ordering_frozen", False)),
            preview_keyframes_created=bool(data.get("preview_keyframes_created", False)),
            motion_blur_disabled=bool(data.get("motion_blur_disabled", True)),
            scene_animation_frozen=bool(data.get("scene_animation_frozen", True)),
            max_position_verification_error=float(data.get("max_position_verification_error", 0.0)),
            max_rotation_verification_error=float(data.get("max_rotation_verification_error", 0.0)),
        )


def _edge_lookup(plan):
    lookup = {}
    for left, right, ratio in plan.get("edges", ()):
        lookup[(min(left, right), max(left, right))] = float(ratio)
    return lookup


def _segment_key(candidate):
    special = "POLAR" if candidate.kind == "polar" else "BRIDGE" if candidate.kind == "bridge" else "COVERAGE" if candidate.provider_type == "COVERAGE_DRIVEN" else "BASE"
    return (candidate.component_index, candidate.layer_index, special)


def _candidate_components(indices, edges):
    remaining = set(indices)
    result = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        stack = [seed]
        component = []
        while stack:
            current = stack.pop()
            component.append(current)
            linked = [other for other in remaining if edges.get((min(current, other), max(current, other)), 0.0) >= 0.10]
            for other in linked:
                remaining.remove(other)
                stack.append(other)
        result.append(sorted(component))
    return result


def _orientation_continuity(left_matrix, right_matrix):
    angle = left_matrix.to_quaternion().rotation_difference(right_matrix.to_quaternion()).angle
    return 1.0 - min(1.0, abs(float(angle)) / math.pi)


def _greedy_component_order(candidates, indices, matrices, edges):
    if not indices:
        return []
    remaining = set(indices)
    current = min(indices, key=lambda index: (
        candidates[index].distance_along, candidates[index].candidate_id
    ))
    order = [current]
    remaining.remove(current)
    while remaining:
        left = candidates[current]
        left_matrix = matrices[left.candidate_id]
        step = max(1e-6, float(left.local_step))
        def key(index):
            candidate = candidates[index]
            ratio = edges.get((min(current, index), max(current, index)), 0.0)
            distance = (left.position - candidate.position).length
            proximity = 1.0 / (1.0 + distance / step)
            continuity = _orientation_continuity(left_matrix, matrices[candidate.candidate_id])
            return (0.50 * ratio + 0.30 * proximity + 0.20 * continuity, ratio, -distance, candidate.candidate_id)
        current = max(remaining, key=key)
        remaining.remove(current)
        order.append(current)
    return order


def ordered_candidate_segments(plan, matrices):
    candidates = list(plan["selected"])
    edges = _edge_lookup(plan)
    grouped = {}
    for index, candidate in enumerate(candidates):
        grouped.setdefault(_segment_key(candidate), []).append(index)
    result = []
    for key in sorted(grouped):
        for component in _candidate_components(grouped[key], edges):
            result.append((key, _greedy_component_order(candidates, component, matrices, edges)))
    return result, edges


def build_sequence(scene, settings, plan, matrices, camera_data, components=(), ordered=True):
    candidates = list(plan["selected"])
    intrinsics = shared_intrinsics(scene, settings, camera_data)
    source_frame = int(getattr(settings, "sequence_source_scene_frame", scene.frame_current))
    if ordered:
        segment_orders, edges = ordered_candidate_segments(plan, matrices)
    else:
        edges = _edge_lookup(plan)
        segment_orders = []
        current_key = None
        current = []
        for index, candidate in enumerate(candidates):
            key = _segment_key(candidate)
            if current and key != current_key:
                segment_orders.append((current_key, current))
                current = []
            current_key = key
            current.append(index)
        if current:
            segment_orders.append((current_key, current))
    frames = []
    segments = []
    logical_id = 0
    for segment_id, (key, indices) in enumerate(segment_orders, 1):
        component_index, layer_index, special = key
        source_name = ""
        if 0 <= component_index < len(components):
            source_name = components[component_index]["object"].name_full
        elif indices:
            source_name = candidates[indices[0]].region_id
        segment_name = f"{source_name or 'region'}_{layer_index}_{special.lower()}_{segment_id:03d}"
        segments.append({
            "segment_id": segment_id,
            "name": segment_name,
            "source_curve": source_name,
            "source_curve_index": int(component_index),
            "layer_index": int(layer_index),
            "sample_type": special,
            "frame_count": len(indices),
        })
        previous_index = None
        for order_in_segment, index in enumerate(indices, 1):
            logical_id += 1
            candidate = candidates[index]
            matrix = matrices[candidate.candidate_id].copy()
            paths = build_output_paths(logical_id, settings)
            overlap = 0.0 if previous_index is None else edges.get((min(previous_index, index), max(previous_index, index)), 0.0)
            sample_type = "POLAR" if candidate.kind == "polar" else "BRIDGE" if candidate.kind == "bridge" else "COVERAGE" if candidate.provider_type == "COVERAGE_DRIVEN" else "BASE"
            frames.append(PoseSample(
                logical_frame_id=logical_id,
                image_id=logical_id,
                segment_id=segment_id,
                order_in_segment=order_in_segment,
                source_curve_name=source_name,
                source_curve_index=int(component_index),
                source_path_distance=float(candidate.distance_along),
                layer_index=int(candidate.layer_index),
                layer_type=str(candidate.layer_name),
                sample_type=sample_type,
                matrix_world=matrix,
                focal_length_mm=float(camera_data.lens),
                sensor_width_mm=float(camera_data.sensor_width),
                sensor_height_mm=float(camera_data.sensor_height),
                sensor_fit=str(camera_data.sensor_fit),
                shift_x=float(camera_data.shift_x),
                shift_y=float(camera_data.shift_y),
                resolution_x=int(settings.resolution_x),
                resolution_y=int(settings.resolution_y),
                horizontal_fov=float(getattr(camera_data, "angle_x", 0.0)),
                vertical_fov=float(getattr(camera_data, "angle_y", 0.0)),
                coverage_score=float(candidate.score),
                overlap_score=float(overlap),
                near_field_class=str(candidate.distance_band),
                is_polar_patch=candidate.kind == "polar",
                is_bridge=candidate.kind == "bridge",
                is_coverage_patch=False,
                rgb_path=paths["rgb"],
                depth_path=paths["depth"],
                normal_path=paths["normal"],
                id_path=paths["id"],
                material_id_path=paths["material_id"],
                object_depth_path=paths["object_depth"],
                object_normal_path=paths["object_normal"],
                mask_path=paths["mask"],
                candidate_id=str(candidate.candidate_id),
                provider_type=str(candidate.provider_type),
                region_id=str(candidate.region_id),
            ))
            previous_index = index
    planning_payload = {
        "candidate_ids": [sample.candidate_id for sample in frames],
        "matrices": [matrix_to_list(sample.matrix_world) for sample in frames],
        "segments": segments,
        "stats": plan.get("stats", {}),
    }
    return PoseSequence(
        source_scene_frame=source_frame,
        shared_intrinsics=intrinsics,
        sequence_intrinsics_hash=intrinsics_hash(intrinsics),
        scene_hash=scene_signature(scene, settings, source_frame),
        planning_hash=_json_hash(planning_payload),
        planning_settings_hash=planning_settings_signature(settings),
        output_settings_hash=output_settings_signature(settings),
        segments=segments,
        frames=frames,
    )


def sequence_path(settings):
    return Path(settings.output_dir) / SEQUENCE_FILENAME


def atomic_write_json(path, payload):
    import time

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    for attempt in range(12):
        try:
            with open(temp, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp, path)
            return
        except (PermissionError, OSError):
            if attempt == 11:
                raise
            time.sleep(0.04 * (attempt + 1))


def save_sequence(scene, settings, sequence, write_disk=True):
    payload = sequence.to_dict()
    text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    scene[SCENE_SEQUENCE_KEY] = text
    if write_disk and getattr(settings, "output_dir", ""):
        atomic_write_json(sequence_path(settings), payload)
    return sequence


def load_sequence(scene, settings, prefer_disk=True):
    scene_payload = None
    raw = scene.get(SCENE_SEQUENCE_KEY, "")
    if raw:
        scene_payload = json.loads(raw)
    disk_payload = None
    path = sequence_path(settings) if getattr(settings, "output_dir", "") else None
    if prefer_disk and path is not None and path.is_file():
        with open(path, "r", encoding="utf-8") as handle:
            disk_payload = json.load(handle)
    if scene_payload and disk_payload:
        if scene_payload.get("planning_hash") != disk_payload.get("planning_hash"):
            raise ValueError(
                "camera_sequence.json belongs to a different scientific plan; "
                "choose an empty output directory or rebuild the pose sequence"
            )
    payload = disk_payload if disk_payload is not None else scene_payload
    return PoseSequence.from_dict(payload) if payload else None


def clear_sequence(scene, settings, remove_disk=False):
    if SCENE_SEQUENCE_KEY in scene:
        del scene[SCENE_SEQUENCE_KEY]
    if remove_disk and getattr(settings, "output_dir", ""):
        try:
            sequence_path(settings).unlink()
        except FileNotFoundError:
            pass


class PoseCameraDataAdapter:
    """Minimal perspective-camera data API used by export and sparse sampling."""

    def __init__(self, sample):
        self._sample = sample
        self.lens = float(sample.focal_length_mm)
        self.sensor_width = float(sample.sensor_width_mm)
        self.sensor_height = float(sample.sensor_height_mm)
        self.sensor_fit = str(sample.sensor_fit)
        self.shift_x = float(sample.shift_x)
        self.shift_y = float(sample.shift_y)
        self.type = "PERSP"
        self.panorama_type = ""
        self.dof = SimpleNamespace(use_dof=False)
        self.animation_data = None

    def view_frame(self, scene=None):
        sample = self._sample
        width = max(1.0, float(sample.resolution_x))
        height = max(1.0, float(sample.resolution_y))
        if self.sensor_fit == "VERTICAL":
            fy = self.lens / max(1e-9, self.sensor_height) * height
            fx = fy
        else:
            fx = self.lens / max(1e-9, self.sensor_width) * width
            fy = fx
        cx = width * 0.5 - self.shift_x * width
        cy = height * 0.5 + self.shift_y * height
        left = -cx / fx
        right = (width - cx) / fx
        bottom = (cy - height) / fy
        top = cy / fy
        return (
            Vector((left, bottom, -1.0)),
            Vector((right, bottom, -1.0)),
            Vector((right, top, -1.0)),
            Vector((left, top, -1.0)),
        )


class PoseCameraAdapter:
    """Camera-like read-only adapter for existing export and sparse-point code."""

    def __init__(self, sample):
        self.sample = sample
        self.name = f"Pose_{sample.logical_frame_id:06d}"
        self.matrix_world = sample.matrix_world.copy()
        self.data = PoseCameraDataAdapter(sample)
        self._props = {
            "gs_patch_camera": bool(sample.is_coverage_patch),
            "gs_dataset_image_stem": Path(sample.rgb_path).stem,
            "gs_scientific_candidate_id": sample.candidate_id,
            "gs_scientific_layer": sample.layer_type,
            "gs_scientific_kind": sample.sample_type.lower(),
            "gs_scientific_provider": sample.provider_type,
            "gs_scientific_region": sample.region_id,
            "gs_scientific_distance_band": sample.near_field_class,
        }

    def get(self, key, default=None):
        return self._props.get(key, default)


def adapters(sequence, include_disabled=False, patch_only=False):
    samples = [
        sample for sample in sequence.frames
        if (include_disabled or sample.render_enabled)
        and (not patch_only or sample.is_coverage_patch)
    ]
    return [PoseCameraAdapter(sample) for sample in samples]


def validate_intrinsics(scene, settings, capture_camera, sequence):
    current = shared_intrinsics(scene, settings, capture_camera.data)
    current_hash = intrinsics_hash(current)
    if current_hash != sequence.sequence_intrinsics_hash:
        raise ValueError(
            "Scientific pose sequence intrinsics changed after planning: "
            f"expected {sequence.sequence_intrinsics_hash}, got {current_hash}"
        )
    for sample in sequence.frames:
        values = dict(sequence.shared_intrinsics)
        values.update({
            "focal_length_mm": float(sample.focal_length_mm),
            "sensor_width_mm": float(sample.sensor_width_mm),
            "sensor_height_mm": float(sample.sensor_height_mm),
            "sensor_fit": str(sample.sensor_fit),
            "shift_x": float(sample.shift_x),
            "shift_y": float(sample.shift_y),
            "resolution_x": int(sample.resolution_x),
            "resolution_y": int(sample.resolution_y),
        })
        if intrinsics_hash(values) != sequence.sequence_intrinsics_hash:
            raise ValueError(f"Pose {sample.logical_frame_id} does not use the shared intrinsics")
    return current_hash


def freeze_sequence(sequence):
    sequence.ordering_frozen = True
    for sample in sequence.frames:
        if sample.render_status == "RENDERING":
            sample.render_status = "PENDING"
    return sequence


def sequence_report(sequence):
    enabled = [sample for sample in sequence.frames if sample.render_enabled]
    pending = [sample for sample in enabled if sample.render_status != "COMPLETE"]
    failed = [sample for sample in enabled if sample.render_status == "FAILED"]
    patch = [sample for sample in enabled if sample.is_coverage_patch]
    return {
        "realization_mode": MODE,
        "blender_camera_object_count": 1,
        "planned_pose_count": len(sequence.frames),
        "render_enabled_pose_count": len(enabled),
        "final_training_frame_count": len(enabled),
        "patch_pose_count": len(patch),
        "sequence_segment_count": len(sequence.segments),
        "sequence_source_scene_frame": sequence.source_scene_frame,
        "sequence_preview_keyframes_created": sequence.preview_keyframes_created,
        "sequence_ordering_method": sequence.ordering_method,
        "sequence_intrinsics_shared": True,
        "sequence_intrinsics_hash": sequence.sequence_intrinsics_hash,
        "sequence_pose_manifest": SEQUENCE_FILENAME,
        "sequence_resume_pending": len(pending),
        "sequence_failed_frames": len(failed),
        "sequence_max_position_verification_error": sequence.max_position_verification_error,
        "sequence_max_rotation_verification_error": sequence.max_rotation_verification_error,
        "scene_animation_frozen": sequence.scene_animation_frozen,
        "motion_blur_disabled": sequence.motion_blur_disabled,
    }


def create_preview_keyframes(scene, capture_camera, sequence):
    clear_preview_keyframes(scene, capture_camera, sequence)
    capture_camera.rotation_mode = "QUATERNION"
    previous = None
    frame_start = max(1, int(sequence.source_scene_frame))
    segment_starts = {segment["segment_id"] for segment in sequence.segments}
    for offset, sample in enumerate((item for item in sequence.frames if item.render_enabled)):
        frame = frame_start + offset
        matrix = sample.matrix_world
        quat = matrix.to_quaternion().normalized()
        if previous is not None and previous.dot(quat) < 0.0:
            quat = Quaternion((-quat.w, -quat.x, -quat.y, -quat.z))
        capture_camera.location = matrix.translation
        capture_camera.rotation_quaternion = quat
        capture_camera.keyframe_insert(data_path="location", frame=frame)
        capture_camera.keyframe_insert(data_path="rotation_quaternion", frame=frame)
        if sample.order_in_segment == 1 and sample.segment_id in segment_starts:
            scene.timeline_markers.new(f"{PREVIEW_MARKER_PREFIX}{sample.segment_id:03d}", frame=frame)
        previous = quat
    action = getattr(getattr(capture_camera, "animation_data", None), "action", None)
    if action:
        fcurves = list(getattr(action, "fcurves", ()))
        if not fcurves:
            for layer in getattr(action, "layers", ()):
                for strip in getattr(layer, "strips", ()):
                    for channelbag in getattr(strip, "channelbags", ()):
                        fcurves.extend(getattr(channelbag, "fcurves", ()))
        for fcurve in fcurves:
            for point in fcurve.keyframe_points:
                point.interpolation = "LINEAR" if fcurve.data_path == "location" else "BEZIER"
    sequence.preview_keyframes_created = True
    return sequence


def clear_preview_keyframes(scene, capture_camera, sequence=None):
    capture_camera.animation_data_clear()
    for marker in list(scene.timeline_markers):
        if marker.name.startswith(PREVIEW_MARKER_PREFIX):
            scene.timeline_markers.remove(marker)
    if sequence is not None:
        sequence.preview_keyframes_created = False
    return sequence


def append_patch_samples(sequence, settings, cameras):
    if sequence.ordering_frozen is False:
        sequence.ordering_frozen = True
    segment_id = max((int(item.get("segment_id", 0)) for item in sequence.segments), default=0) + 1
    start_logical = max((sample.logical_frame_id for sample in sequence.frames), default=0)
    start_image = max((sample.image_id for sample in sequence.frames), default=0)
    patch_start = sum(sample.is_coverage_patch for sample in sequence.frames)
    added = []
    for offset, camera in enumerate(cameras, 1):
        patch_index = patch_start + offset
        logical_id = start_logical + offset
        paths = build_output_paths(logical_id, settings, patch_index=patch_index)
        sample = PoseSample(
            logical_frame_id=logical_id,
            image_id=start_image + offset,
            segment_id=segment_id,
            order_in_segment=offset,
            source_curve_name="Coverage Patch",
            source_curve_index=-1,
            source_path_distance=0.0,
            layer_index=0,
            layer_type=str(camera.get("gs_patch_region", "Patch")),
            sample_type="COVERAGE_PATCH",
            matrix_world=camera.matrix_world.copy(),
            focal_length_mm=float(sequence.shared_intrinsics["focal_length_mm"]),
            sensor_width_mm=float(sequence.shared_intrinsics["sensor_width_mm"]),
            sensor_height_mm=float(sequence.shared_intrinsics["sensor_height_mm"]),
            sensor_fit=str(sequence.shared_intrinsics["sensor_fit"]),
            shift_x=float(sequence.shared_intrinsics["shift_x"]),
            shift_y=float(sequence.shared_intrinsics["shift_y"]),
            resolution_x=int(sequence.shared_intrinsics["resolution_x"]),
            resolution_y=int(sequence.shared_intrinsics["resolution_y"]),
            horizontal_fov=float(getattr(camera.data, "angle_x", 0.0)),
            vertical_fov=float(getattr(camera.data, "angle_y", 0.0)),
            coverage_score=float(camera.get("gs_patch_score", 0.0)),
            overlap_score=0.0,
            near_field_class="PATCH",
            is_coverage_patch=True,
            rgb_path=paths["rgb"],
            depth_path=paths["depth"],
            normal_path=paths["normal"],
            id_path=paths["id"],
            material_id_path=paths["material_id"],
            object_depth_path=paths["object_depth"],
            object_normal_path=paths["object_normal"],
            mask_path=paths["mask"],
            candidate_id=str(camera.get("gs_patch_candidate_id", f"PATCH_{patch_index:06d}")),
            provider_type="COVERAGE_PATCH",
            region_id=str(camera.get("gs_patch_region", "Patch")),
        )
        sequence.frames.append(sample)
        added.append(sample)
    if added:
        sequence.segments.append({
            "segment_id": segment_id,
            "name": f"coverage_patch_{segment_id:03d}",
            "source_curve": "Coverage Patch",
            "source_curve_index": -1,
            "layer_index": 0,
            "sample_type": "COVERAGE_PATCH",
            "frame_count": len(added),
        })
    return added
