from dataclasses import dataclass
from pathlib import Path

import numpy as np


CONVERTIBLE_OBJECT_TYPES = {"MESH", "CURVE", "SURFACE", "FONT", "META", "CURVES"}

@dataclass
class MeshInstance:
    key: str
    object_name: str
    source_name: str
    material_names: list
    evaluated_object: object
    matrix_world: object
    persistent_id: tuple
    depsgraph: object


def enumerate_mesh_instances(scene, settings):
    import bpy

    depsgraph = bpy.context.evaluated_depsgraph_get()
    selected_collection = getattr(settings, "mesh_collection", None)
    selected_names = set()
    if selected_collection is not None:
        selected_names = {obj.name for obj in selected_collection.all_objects}
    result = []
    seen = set()
    for instance in depsgraph.object_instances:
        obj = instance.object
        original = getattr(obj, "original", obj)
        if obj.type not in CONVERTIBLE_OBJECT_TYPES or not hasattr(obj, "to_mesh"):
            continue
        if instance.is_instance and instance.parent is not None:
            parent_original = getattr(instance.parent, "original", instance.parent)
            if parent_original.name == original.name:
                # Blender exposes a Curve and its internal evaluated Mesh proxy separately.
                # The Curve's to_mesh() already contains that geometry.
                continue
        if selected_names and original.name not in selected_names:
            continue
        if getattr(settings, "exclude_hidden", True) and not original.visible_get():
            continue
        if getattr(settings, "exclude_render_disabled", True) and original.hide_render:
            continue
        persistent_id = tuple(value for value in instance.persistent_id if value != 2147483647)
        if not instance.is_instance:
            persistent_id = ()
        key = (original.name, persistent_id, tuple(round(value, 8) for row in instance.matrix_world for value in row))
        if key in seen:
            continue
        seen.add(key)
        suffix = "_".join(str(value) for value in persistent_id) or "base"
        result.append(MeshInstance(
            key=f"{len(result) + 1:06d}_{original.name}_{suffix}",
            object_name=original.name,
            source_name=getattr(getattr(original, "data", None), "name", original.name),
            material_names=[slot.material.name if slot.material else "" for slot in original.material_slots],
            evaluated_object=obj,
            matrix_world=instance.matrix_world.copy(),
            persistent_id=persistent_id,
            depsgraph=depsgraph,
        ))
    return result


def _transform_points(points, matrix):
    transform = np.asarray([[float(value) for value in row] for row in matrix], dtype=np.float64)
    homogeneous = np.concatenate((points.astype(np.float64), np.ones((len(points), 1))), axis=1)
    return (homogeneous @ transform.T)[:, :3].astype(np.float32)


def _normal_matrix(matrix):
    transform = np.asarray([[float(value) for value in row[:3]] for row in matrix][:3], dtype=np.float64)
    return np.linalg.inv(transform).T


def _normalize(values):
    lengths = np.linalg.norm(values, axis=-1, keepdims=True)
    return values / np.maximum(lengths, 1e-12)


def extract_mesh_instance(instance, output_path, object_id):
    """Write one dependency-graph evaluated mesh instance as a compact NPZ part."""
    obj = instance.evaluated_object
    mesh = obj.to_mesh(preserve_all_data_layers=True, depsgraph=instance.depsgraph)
    try:
        mesh.calc_loop_triangles()
        if not mesh.loop_triangles:
            raise ValueError(f"对象 {instance.object_name} 没有可采样三角形")
        vertices = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
        mesh.vertices.foreach_get("co", vertices)
        vertices = vertices.reshape(-1, 3)
        vertices = _transform_points(vertices, instance.matrix_world)

        vertex_indices = np.empty(len(mesh.loop_triangles) * 3, dtype=np.int32)
        loop_indices = np.empty(len(mesh.loop_triangles) * 3, dtype=np.int32)
        mesh.loop_triangles.foreach_get("vertices", vertex_indices)
        mesh.loop_triangles.foreach_get("loops", loop_indices)
        vertex_indices = vertex_indices.reshape(-1, 3)
        loop_indices = loop_indices.reshape(-1, 3)
        triangles = vertices[vertex_indices]

        loop_normals = np.empty(len(mesh.loops) * 3, dtype=np.float32)
        try:
            mesh.corner_normals.foreach_get("vector", loop_normals)
        except (AttributeError, TypeError):
            mesh.loops.foreach_get("normal", loop_normals)
        loop_normals = loop_normals.reshape(-1, 3)
        normals = loop_normals[loop_indices] @ _normal_matrix(instance.matrix_world).T
        normals = _normalize(normals).astype(np.float32)

        uvs = np.zeros((len(triangles), 3, 2), dtype=np.float32)
        uv_layer = mesh.uv_layers.active
        if uv_layer is not None:
            loop_uvs = np.empty(len(mesh.loops) * 2, dtype=np.float32)
            uv_layer.data.foreach_get("uv", loop_uvs)
            uvs = loop_uvs.reshape(-1, 2)[loop_indices]

        material_indices = np.zeros(len(triangles), dtype=np.int32)
        polygon_materials = np.empty(len(mesh.polygons), dtype=np.int32)
        if len(mesh.polygons):
            mesh.polygons.foreach_get("material_index", polygon_materials)
            polygon_indices = np.empty(len(mesh.loop_triangles), dtype=np.int32)
            mesh.loop_triangles.foreach_get("polygon_index", polygon_indices)
            material_indices = polygon_materials[polygon_indices]

        cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        areas = (np.linalg.norm(cross, axis=1) * 0.5).astype(np.float32)
        valid = areas > 1e-12
        if not np.any(valid):
            raise ValueError(f"对象 {instance.object_name} 的三角形面积均为零")
        payload = {
            "triangles": triangles[valid],
            "normals": normals[valid],
            "uvs": uvs[valid],
            "areas": areas[valid],
            "triangle_ids": np.arange(np.count_nonzero(valid), dtype=np.int64),
            "material_ids": material_indices[valid].astype(np.int32),
            "object_ids": np.full(np.count_nonzero(valid), int(object_id), dtype=np.int32),
        }
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(output_path, **payload)
        return {
            "key": instance.key,
            "object_name": instance.object_name,
            "source_name": instance.source_name,
            "persistent_id": list(instance.persistent_id),
            "object_id": int(object_id),
            "material_names": instance.material_names,
            "triangle_count": int(np.count_nonzero(valid)),
            "zero_area_triangle_count": int(len(valid) - np.count_nonzero(valid)),
            "surface_area": float(areas[valid].sum(dtype=np.float64)),
            "has_uv": uv_layer is not None,
            "part_file": output_path.name,
        }
    finally:
        obj.to_mesh_clear()


def _triangle_frames(triangles):
    edge_1 = triangles[:, 1] - triangles[:, 0]
    edge_2 = triangles[:, 2] - triangles[:, 0]
    normals = _normalize(np.cross(edge_1, edge_2))
    tangents = edge_1 - normals * np.sum(edge_1 * normals, axis=1, keepdims=True)
    invalid = np.linalg.norm(tangents, axis=1) < 1e-8
    if np.any(invalid):
        fallback = np.cross(normals[invalid], np.asarray([1.0, 0.0, 0.0], dtype=np.float32))
        also_invalid = np.linalg.norm(fallback, axis=1) < 1e-8
        fallback[also_invalid] = np.cross(
            normals[invalid][also_invalid], np.asarray([0.0, 1.0, 0.0], dtype=np.float32)
        )
        tangents[invalid] = fallback
    tangents = _normalize(tangents)
    bitangents = _normalize(np.cross(normals, tangents))
    return normals.astype(np.float32), tangents.astype(np.float32), bitangents.astype(np.float32)


def merge_training_mesh(part_paths, metadata, output_path, world_unit_to_meters=1.0):
    """Create a stable training topology whose row IDs match Gaussian anchors."""
    triangle_positions = []
    corner_normals = []
    triangle_uvs = []
    triangle_areas = []
    object_ids = []
    material_ids = []
    handles = []
    try:
        for path in part_paths:
            handle = np.load(path, allow_pickle=False)
            handles.append(handle)
            triangle_positions.append(np.asarray(handle["triangles"], dtype=np.float32))
            corner_normals.append(np.asarray(handle["normals"], dtype=np.float32))
            triangle_uvs.append(np.asarray(handle["uvs"], dtype=np.float32))
            triangle_areas.append(np.asarray(handle["areas"], dtype=np.float32))
            object_ids.append(np.asarray(handle["object_ids"], dtype=np.int32))
            material_ids.append(np.asarray(handle["material_ids"], dtype=np.int32))
        positions = np.concatenate(triangle_positions, axis=0)
        normals_per_corner = np.concatenate(corner_normals, axis=0)
        uvs = np.concatenate(triangle_uvs, axis=0)
        count = len(positions)
        vertices = positions.reshape(-1, 3)
        triangles = np.arange(count * 3, dtype=np.int64).reshape(count, 3)
        face_normals, tangents, bitangents = _triangle_frames(positions)
        payload = {
            "schema_version": np.asarray("1.0"),
            "coordinate_system": np.asarray("Blender world: +X right, +Y forward, +Z up"),
            "world_unit_to_meters": np.asarray(float(world_unit_to_meters), dtype=np.float64),
            "vertices_world": vertices.astype(np.float32),
            "triangles": triangles,
            "triangle_normals_world": face_normals,
            "triangle_object_ids": np.concatenate(object_ids).astype(np.int32),
            "triangle_material_ids": np.concatenate(material_ids).astype(np.int32),
            "triangle_tangents_world": tangents,
            "triangle_bitangents_world": bitangents,
            "vertex_normals_world": normals_per_corner.reshape(-1, 3).astype(np.float32),
            "triangle_uvs": uvs.astype(np.float32),
            "triangle_areas": np.concatenate(triangle_areas).astype(np.float32),
            "object_names": np.asarray([item["object_name"] for item in metadata], dtype=np.str_),
            "material_names": np.asarray(
                sorted({name for item in metadata for name in item["material_names"] if name}), dtype=np.str_
            ),
        }
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(output_path, **payload)
        return {"path": str(output_path), "triangle_count": int(count), "vertex_count": int(len(vertices))}
    finally:
        for handle in handles:
            handle.close()

