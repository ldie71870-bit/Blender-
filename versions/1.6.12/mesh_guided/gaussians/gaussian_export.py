from pathlib import Path

import numpy as np

from .gaussian_initializer import initialize_gaussians
from ..utils.json_io import atomic_write_json


def _load_sample_parts(paths):
    parts = []
    handles = []
    try:
        for path in paths:
            handle = np.load(path, allow_pickle=False)
            handles.append(handle)
            parts.append({key: handle[key] for key in handle.files})
        keys = parts[0].keys()
        return {key: np.concatenate([part[key] for part in parts], axis=0) for key in keys}
    finally:
        for handle in handles:
            handle.close()


def write_gaussian_ply(path, gaussians):
    path = Path(path)
    count = len(gaussians["means"])
    dtype = np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
        ("nx", "<f4"), ("ny", "<f4"), ("nz", "<f4"),
        ("f_dc_0", "<f4"), ("f_dc_1", "<f4"), ("f_dc_2", "<f4"),
        ("opacity", "<f4"),
        ("scale_0", "<f4"), ("scale_1", "<f4"), ("scale_2", "<f4"),
        ("rot_0", "<f4"), ("rot_1", "<f4"), ("rot_2", "<f4"), ("rot_3", "<f4"),
    ])
    data = np.empty(count, dtype=dtype)
    for names, source in (
        (("x", "y", "z"), gaussians["means"]),
        (("nx", "ny", "nz"), gaussians["surface_normals"]),
        (("f_dc_0", "f_dc_1", "f_dc_2"), gaussians["sh0"]),
        (("scale_0", "scale_1", "scale_2"), gaussians["log_scales"]),
        (("rot_0", "rot_1", "rot_2", "rot_3"), gaussians["quats"]),
    ):
        for column, name in enumerate(names):
            data[name] = source[:, column]
    data["opacity"] = gaussians["opacities"]
    header = ["ply", "format binary_little_endian 1.0", f"element vertex {count}"]
    for name in dtype.names:
        header.append(f"property float {name}")
    header.extend(("end_header", ""))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as handle:
        handle.write("\n".join(header).encode("ascii"))
        data.tofile(handle)


def merge_and_export(sample_paths, output_dir, normal_thickness_ratio=0.05, compressed=True,
                     world_unit_to_meters=1.0):
    if not sample_paths:
        raise ValueError("没有可合并的 Mesh 采样分块")
    samples = _load_sample_parts(sample_paths)
    gaussians = initialize_gaussians(samples, normal_thickness_ratio=normal_thickness_ratio)
    gaussians.update({
        "schema_version": np.asarray("1.0"),
        "coordinate_system": np.asarray("Blender world: +X right, +Y forward, +Z up"),
        "world_unit_to_meters": np.asarray(float(world_unit_to_meters), dtype=np.float64),
        "quaternion_order": np.asarray("wxyz"),
        "scale_encoding": np.asarray("linear_world"),
        "opacity_encoding": np.asarray("logit"),
        "color_encoding": np.asarray("linear_rgb"),
        "sh_encoding": np.asarray("3dgs_real_sh"),
        "surface_normal_space": np.asarray("world"),
        "gaussian_normal_axis": np.asarray("local_z"),
    })
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    npz_path = output_dir / "init_gaussians.npz"
    if compressed:
        np.savez_compressed(npz_path, **gaussians)
    else:
        np.savez(npz_path, **gaussians)
    ply_path = output_dir / "init_gaussians.ply"
    write_gaussian_ply(ply_path, gaussians)
    metadata_path = output_dir / "init_gaussians.metadata.json"
    atomic_write_json(metadata_path, {
        "schema_version": "1.0",
        "coordinate_system": "Blender world: +X right, +Y forward, +Z up",
        "world_unit_to_meters": float(world_unit_to_meters),
        "quaternion_order": "wxyz",
        "scale_encoding": "linear_world",
        "opacity_encoding": "logit",
        "color_encoding": "linear_rgb",
        "sh_encoding": "3dgs_real_sh",
        "surface_normal_space": "world",
        "gaussian_normal_axis": "local_z",
        "count": int(len(gaussians["means"])),
    })
    return {
        "count": int(len(gaussians["means"])),
        "npz": str(npz_path),
        "ply": str(ply_path),
        "metadata": str(metadata_path),
    }

