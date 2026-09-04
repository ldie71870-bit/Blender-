from pathlib import Path

import numpy as np

from .cameras.coordinate_conversion import matrices_are_inverse
from .utils.json_io import atomic_write_json, read_json


def validate_initial_gaussians(path):
    errors = []
    required = {"means", "quats", "scales", "opacities", "sh0", "triangle_ids", "barycentrics"}
    with np.load(path, allow_pickle=False) as data:
        missing = sorted(required - set(data.files))
        if missing:
            errors.append("初始高斯缺少字段: " + ", ".join(missing))
            return errors, 0
        count = len(data["means"])
        for key in required:
            if len(data[key]) != count:
                errors.append(f"字段 {key} 数量与 means 不一致")
        for key in ("means", "quats", "scales", "opacities", "sh0", "barycentrics"):
            if not np.all(np.isfinite(data[key])):
                errors.append(f"字段 {key} 包含 NaN 或 Inf")
        if not np.all(data["scales"] > 0):
            errors.append("高斯尺度必须全部为正")
        quaternion_norm = np.linalg.norm(data["quats"], axis=1)
        if not np.allclose(quaternion_norm, 1.0, atol=1e-4):
            errors.append("高斯四元数未归一化")
        if not np.allclose(data["barycentrics"].sum(axis=1), 1.0, atol=1e-5):
            errors.append("重心坐标之和不为 1")
    return errors, count


def _read_blender_image(path):
    import bpy

    image = bpy.data.images.load(str(path), check_existing=False)
    try:
        width, height = int(image.size[0]), int(image.size[1])
        pixels = np.empty(width * height * 4, dtype=np.float32)
        image.pixels.foreach_get(pixels)
        return pixels.reshape(height, width, 4), width, height
    finally:
        bpy.data.images.remove(image)


def _numbered_file(directory, camera_id, suffixes):
    stem = f"{int(camera_id):06d}"
    for suffix in suffixes:
        path = directory / (stem + suffix)
        if path.is_file():
            return path
    return None


def _validate_ground_truth(root, records, validation_ids, errors, triangle_count):
    for record in records:
        camera_id = int(record["id"])
        split = "validation" if camera_id in validation_ids else "train"
        width = int(record["width"])
        height = int(record["height"])

        rgb_path = _numbered_file(root / "images" / split, camera_id, (".png", ".jpg", ".jpeg", ".exr"))
        depth_path = _numbered_file(root / "depth" / split, camera_id, (".exr", ".npy"))
        normal_path = _numbered_file(root / "normal" / split, camera_id, (".exr", ".npy"))
        object_path = _numbered_file(root / "object_id" / split, camera_id, (".npy", ".exr"))
        triangle_path = _numbered_file(root / "triangle_id" / split, camera_id, (".npy",))

        if rgb_path is not None:
            rgba, actual_width, actual_height = _read_blender_image(rgb_path)
            if (actual_width, actual_height) != (width, height):
                errors.append(f"RGB {camera_id:06d} 尺寸 {actual_width}x{actual_height} 与相机记录 {width}x{height} 不一致")
            if not np.all(np.isfinite(rgba)):
                errors.append(f"RGB {camera_id:06d} 包含 NaN 或 Inf")

        if depth_path is not None and depth_path.suffix.lower() == ".exr":
            rgba, actual_width, actual_height = _read_blender_image(depth_path)
            if (actual_width, actual_height) != (width, height):
                errors.append(f"Depth {camera_id:06d} 尺寸不一致")
            mask = rgba[:, :, 3] > 0.5
            if np.any(mask) and not np.all(np.isfinite(rgba[:, :, 0][mask])):
                errors.append(f"Depth {camera_id:06d} 的有效像素包含 NaN 或 Inf")
            if np.any(mask) and not np.all(rgba[:, :, 0][mask] > 0.0):
                errors.append(f"Depth {camera_id:06d} 的有效深度必须为正")

        if normal_path is not None and normal_path.suffix.lower() == ".exr":
            rgba, actual_width, actual_height = _read_blender_image(normal_path)
            if (actual_width, actual_height) != (width, height):
                errors.append(f"Normal {camera_id:06d} 尺寸不一致")
            mask = rgba[:, :, 3] > 0.99
            if np.any(mask):
                normals = rgba[:, :, :3][mask] * 2.0 - 1.0
                lengths = np.linalg.norm(normals, axis=1)
                if not np.all(np.isfinite(lengths)) or not np.allclose(lengths, 1.0, atol=0.08):
                    errors.append(f"Normal {camera_id:06d} 包含未归一化或非有限法线")

        if object_path is not None and object_path.suffix.lower() == ".npy":
            object_ids = np.load(object_path, allow_pickle=False)
            if object_ids.dtype != np.int32:
                errors.append(f"Object ID {camera_id:06d} 必须为 int32")
            if object_ids.shape != (height, width):
                errors.append(f"Object ID {camera_id:06d} 形状 {object_ids.shape} 与 {(height, width)} 不一致")
            if object_ids.size and int(object_ids.min()) < -1:
                errors.append(f"Object ID {camera_id:06d} 包含小于 -1 的无效值")

        if triangle_path is not None:
            triangle_ids = np.load(triangle_path, allow_pickle=False)
            if triangle_ids.dtype != np.int64:
                errors.append(f"Triangle ID {camera_id:06d} 必须为 int64")
            if triangle_ids.shape != (height, width):
                errors.append(f"Triangle ID {camera_id:06d} 形状 {triangle_ids.shape} 与 {(height, width)} 不一致")
            if triangle_ids.size and int(triangle_ids.min()) < -1:
                errors.append(f"Triangle ID {camera_id:06d} 包含小于 -1 的无效值")
            valid_ids = triangle_ids[triangle_ids >= 0]
            if valid_ids.size and int(valid_ids.max()) >= triangle_count:
                errors.append(f"Triangle ID {camera_id:06d} 超出 training_mesh.npz 三角形数量 {triangle_count}")

def _validate_training_mesh(root, errors):
    path = root / "mesh" / "training_mesh.npz"
    if not path.is_file():
        errors.append(f"缺少稳定训练拓扑: {path}")
        return 0
    required = {
        "vertices_world", "triangles", "triangle_normals_world",
        "triangle_object_ids", "triangle_material_ids",
        "triangle_tangents_world", "triangle_bitangents_world",
    }
    with np.load(path, allow_pickle=False) as mesh:
        missing = sorted(required - set(mesh.files))
        if missing:
            errors.append("training_mesh.npz 缺少字段: " + ", ".join(missing))
            return 0
        triangles = mesh["triangles"]
        vertices = mesh["vertices_world"]
        count = len(triangles)
        if triangles.ndim != 2 or triangles.shape[1] != 3:
            errors.append("training_mesh.npz triangles 必须为 [F,3]")
        elif triangles.size and (int(triangles.min()) < 0 or int(triangles.max()) >= len(vertices)):
            errors.append("training_mesh.npz 包含越界顶点索引")
        for name in required - {"vertices_world", "triangles"}:
            if len(mesh[name]) != count:
                errors.append(f"training_mesh.npz {name} 数量与 triangles 不一致")
    gaussian_path = root / "gaussians" / "init_gaussians.npz"
    if gaussian_path.is_file():
        with np.load(gaussian_path, allow_pickle=False) as gaussians:
            triangle_ids = gaussians["triangle_ids"]
            if triangle_ids.size and (int(triangle_ids.min()) < 0 or int(triangle_ids.max()) >= count):
                errors.append("初始高斯 triangle_ids 与 training_mesh.npz 不匹配")
    return count
def validate_dataset(root, expect_images=False):
    root = Path(root)
    errors = []
    warnings = []
    training_triangle_count = _validate_training_mesh(root, errors)
    gaussian_errors, gaussian_count = validate_initial_gaussians(root / "gaussians" / "init_gaussians.npz")
    errors.extend(gaussian_errors)
    cameras = read_json(root / "cameras" / "cameras.json", {}) or {}
    records = cameras.get("cameras", [])
    for item in records:
        if not matrices_are_inverse(item["camera_to_world"], item["world_to_camera"]):
            errors.append(f"相机矩阵不互逆: {item.get('name', item.get('id'))}")
    counts = {}
    for kind, suffixes in (("images", (".png", ".jpg", ".jpeg", ".exr")),
                           ("depth", (".exr", ".npy")),
                           ("normal", (".exr", ".npy")),
                           ("object_id", (".npy", ".exr")),
                           ("triangle_id", (".npy",))):
        files = [path for split in ("train", "validation") for path in (root / kind / split).glob("*")
                 if path.is_file() and path.suffix.lower() in suffixes]
        counts[kind] = len(files)
    if expect_images:
        expected = len(records)
        for kind in ("images", "depth", "normal", "object_id", "triangle_id"):
            if counts[kind] != expected:
                errors.append(f"{kind} 数量 {counts[kind]} 与相机数量 {expected} 不一致")
        validation = read_json(root / "cameras" / "validation.json", {}) or {}
        validation_ids = {int(item["id"]) for item in validation.get("cameras", [])}
        _validate_ground_truth(root, records, validation_ids, errors, training_triangle_count)
    report = {
        "valid": not errors,
        "errors": errors,
        "warnings": warnings,
        "initial_gaussian_count": gaussian_count,
        "training_triangle_count": training_triangle_count,
        "camera_count": len(records),
        "output_counts": counts,
        "checked_frame_count": len(records) if expect_images else 0,
    }
    atomic_write_json(root / "dataset_validation.json", report)
    return report

