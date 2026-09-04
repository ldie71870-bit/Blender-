from pathlib import Path

import numpy as np


def allocate_sample_counts(metadata, target_count, minimum_per_object=32, small_object_weight=1.0):
    count = len(metadata)
    if not count:
        return []
    target_count = max(count, int(target_count))
    areas = np.asarray([max(0.0, float(item["surface_area"])) for item in metadata], dtype=np.float64)
    if not np.any(areas > 0):
        areas[:] = 1.0
    median = float(np.median(areas[areas > 0]))
    boost = np.maximum(1.0, median / np.maximum(areas, 1e-12)) ** max(0.0, float(small_object_weight))
    effective = areas * np.minimum(boost, 8.0)
    base = min(int(minimum_per_object), max(1, target_count // count))
    result = np.full(count, base, dtype=np.int64)
    remaining = max(0, target_count - int(result.sum()))
    exact = effective / effective.sum() * remaining
    result += np.floor(exact).astype(np.int64)
    remainder = target_count - int(result.sum())
    if remainder:
        fractional = exact - np.floor(exact)
        for index in np.argsort(-fractional)[:remainder]:
            result[index] += 1
    return [int(value) for value in result]


def _normalize(values):
    lengths = np.linalg.norm(values, axis=-1, keepdims=True)
    return values / np.maximum(lengths, 1e-12)


def sample_triangles(triangles, normals, uvs, areas, count, seed=0, curvature_weight=0.0,
                     texture_variation=None, texture_weight=0.0):
    triangles = np.asarray(triangles, dtype=np.float32)
    normals = np.asarray(normals, dtype=np.float32)
    uvs = np.asarray(uvs, dtype=np.float32)
    areas = np.asarray(areas, dtype=np.float64)
    variation = 1.0 - np.clip(np.abs((normals * normals.mean(axis=1, keepdims=True)).sum(axis=2)).mean(axis=1), 0.0, 1.0)
    weights = areas * (1.0 + max(0.0, float(curvature_weight)) * variation)
    if texture_variation is not None and texture_weight > 0.0:
        texture_variation = np.asarray(texture_variation, dtype=np.float64)
        weights *= 1.0 + max(0.0, float(texture_weight)) * np.maximum(texture_variation, 0.0)
    weights = weights / weights.sum()
    rng = np.random.default_rng(int(seed))
    selected = rng.choice(len(triangles), size=int(count), replace=True, p=weights)
    first = rng.random(int(count), dtype=np.float32)
    second = rng.random(int(count), dtype=np.float32)
    sqrt_first = np.sqrt(first)
    barycentrics = np.column_stack((1.0 - sqrt_first, sqrt_first * (1.0 - second), sqrt_first * second)).astype(np.float32)
    chosen_triangles = triangles[selected]
    chosen_normals = normals[selected]
    chosen_uvs = uvs[selected]
    positions = np.einsum("ni,nij->nj", barycentrics, chosen_triangles)
    surface_normals = _normalize(np.einsum("ni,nij->nj", barycentrics, chosen_normals)).astype(np.float32)
    sample_uvs = np.einsum("ni,nij->nj", barycentrics, chosen_uvs).astype(np.float32)
    edge = chosen_triangles[:, 1] - chosen_triangles[:, 0]
    tangents = edge - surface_normals * np.sum(edge * surface_normals, axis=1, keepdims=True)
    invalid = np.linalg.norm(tangents, axis=1) < 1e-8
    if np.any(invalid):
        fallback = np.cross(surface_normals[invalid], np.asarray([1.0, 0.0, 0.0], dtype=np.float32))
        also_invalid = np.linalg.norm(fallback, axis=1) < 1e-8
        fallback[also_invalid] = np.cross(surface_normals[invalid][also_invalid], np.asarray([0.0, 1.0, 0.0], dtype=np.float32))
        tangents[invalid] = fallback
    tangents = _normalize(tangents).astype(np.float32)
    bitangents = _normalize(np.cross(surface_normals, tangents)).astype(np.float32)
    local_area = np.maximum(areas[selected] / np.maximum(1.0, count * weights[selected]), 1e-12)
    spacing = np.sqrt(local_area).astype(np.float32)
    return {
        "positions": positions.astype(np.float32),
        "surface_normals": surface_normals,
        "tangents": tangents,
        "bitangents": bitangents,
        "uvs": sample_uvs,
        "barycentrics": barycentrics,
        "selected_triangles": selected.astype(np.int64),
        "spacing": spacing,
    }


def sample_mesh_part(mesh_part_path, output_path, count, material_sampler, seed=0, curvature_weight=0.0,
                     texture_weight=0.0, triangle_id_offset=0):
    with np.load(mesh_part_path, allow_pickle=False) as part:
        texture_variation = None
        if texture_weight > 0.0:
            triangle_count = len(part["material_ids"])
            vertex_material_ids = np.repeat(part["material_ids"], 3)
            vertex_colors = material_sampler.sample(vertex_material_ids, part["uvs"].reshape(-1, 2))["colors"]
            vertex_colors = vertex_colors.reshape(triangle_count, 3, 3)
            texture_variation = np.linalg.norm(vertex_colors.max(axis=1) - vertex_colors.min(axis=1), axis=1)
        sampled = sample_triangles(
            part["triangles"], part["normals"], part["uvs"], part["areas"], count,
            seed=seed, curvature_weight=curvature_weight,
            texture_variation=texture_variation, texture_weight=texture_weight,
        )
        selected = sampled.pop("selected_triangles")
        material_ids = part["material_ids"][selected].astype(np.int32)
        sampled["material_ids"] = material_ids
        sampled["object_ids"] = part["object_ids"][selected].astype(np.int32)
        sampled["triangle_ids"] = (part["triangle_ids"][selected] + int(triangle_id_offset)).astype(np.int64)
        sampled.update(material_sampler.sample(material_ids, sampled["uvs"]))
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output_path, **sampled)
    return {"sample_count": int(count), "part_file": output_path.name}

