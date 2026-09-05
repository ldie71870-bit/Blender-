import math
from pathlib import Path

import numpy as np

from ..utils.json_io import atomic_write_json
from .coordinate_conversion import camera_to_world_opencv, world_to_camera_opencv


def _matrix_numpy(matrix):
    return np.asarray([[float(value) for value in row] for row in matrix], dtype=np.float64)


def camera_intrinsics(scene, camera, width=None, height=None):
    width = int(width or scene.render.resolution_x * scene.render.resolution_percentage / 100)
    height = int(height or scene.render.resolution_y * scene.render.resolution_percentage / 100)
    data = camera.data
    fit = data.sensor_fit
    pixel_aspect = float(scene.render.pixel_aspect_y) / max(1e-9, float(scene.render.pixel_aspect_x))
    if fit == "AUTO":
        fit = "HORIZONTAL" if pixel_aspect * width >= height else "VERTICAL"
    sensor_size = float(data.sensor_height if fit == "VERTICAL" else data.sensor_width)
    view_factor = pixel_aspect * height if fit == "VERTICAL" else width
    pixel_size_mm = sensor_size / max(1e-9, float(data.lens)) / max(1.0, view_factor)
    fx = 1.0 / pixel_size_mm
    fy = fx / pixel_aspect
    cx = width * 0.5 - float(data.shift_x) * view_factor
    cy = height * 0.5 + float(data.shift_y) * view_factor / pixel_aspect
    return {
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "width": width,
        "height": height,
        "sensor_width_mm": float(data.sensor_width),
        "sensor_height_mm": float(data.sensor_height),
        "focal_length_mm": float(data.lens),
    }


def camera_record(scene, camera, index, width=None, height=None):
    blender_c2w = _matrix_numpy(camera.matrix_world)
    c2w = camera_to_world_opencv(blender_c2w)
    w2c = world_to_camera_opencv(blender_c2w)
    record = {
        "id": int(index),
        "name": camera.name,
        "frame": int(scene.frame_current),
        "image": f"{int(index):06d}.png",
        "camera_to_world": c2w.tolist(),
        "world_to_camera": w2c.tolist(),
        "blender_camera_to_world": blender_c2w.tolist(),
        "near": float(camera.data.clip_start),
        "far": float(camera.data.clip_end),
        "model": "PINHOLE" if camera.data.type != "PANO" else "PANORAMA",
    }
    record.update(camera_intrinsics(scene, camera, width=width, height=height))
    return record


def export_cameras(scene, cameras, root, validation_ratio=0.1, width=None, height=None,
                   world_unit_to_meters=1.0):
    root = Path(root)
    records = [camera_record(scene, camera, index, width, height)
               for index, camera in enumerate(cameras, 1)]
    validation_count = int(math.ceil(len(records) * max(0.0, min(0.5, validation_ratio))))
    validation_ids = set()
    if validation_count:
        stride = max(1, len(records) // validation_count)
        validation_ids = {records[index]["id"] for index in range(stride - 1, len(records), stride)}
        validation_ids = set(sorted(validation_ids)[:validation_count])
    for item in records:
        camera_id = int(item["id"])
        split = "validation" if camera_id in validation_ids else "train"
        stem = f"{camera_id:06d}"
        item.update({
            "camera_id": camera_id,
            "file_id": stem,
            "image_path": f"images/{split}/{stem}.png",
            "depth_path": f"depth/{split}/{stem}.exr",
            "normal_path": f"normal/{split}/{stem}.exr",
            "object_id_path": f"object_id/{split}/{stem}.npy",
            "triangle_id_path": f"triangle_id/{split}/{stem}.npy",
            "color_space": "sRGB",
            "depth_type": "camera_positive_z",
            "normal_space": "camera",
            "coordinate_convention": "OpenCV: +X right, +Y down, +Z forward",
            "world_unit_to_meters": float(world_unit_to_meters),
        })
    train = [item for item in records if item["id"] not in validation_ids]
    validation = [item for item in records if item["id"] in validation_ids]
    payload = {
        "schema_version": "1.0",
        "world_unit_to_meters": float(world_unit_to_meters),
        "coordinate_system": "OpenCV: +X right, +Y down, +Z forward",
        "source_coordinate_system": "Blender: +X right, +Y up, -Z forward",
        "camera_count": len(records),
        "cameras": records,
    }
    atomic_write_json(root / "cameras" / "cameras.json", payload)
    atomic_write_json(root / "cameras" / "train.json", {"schema_version": "1.0", "cameras": train})
    atomic_write_json(root / "cameras" / "validation.json", {"schema_version": "1.0", "cameras": validation})
    return records, train, validation

