from pathlib import Path

import numpy as np


def _normal_material():
    import bpy

    material = bpy.data.materials.new("GS_MeshGuided_Normal_Override")
    material.use_nodes = True
    nodes = material.node_tree.nodes
    nodes.clear()
    output = nodes.new("ShaderNodeOutputMaterial")
    geometry = nodes.new("ShaderNodeNewGeometry")
    transform = nodes.new("ShaderNodeVectorTransform")
    transform.vector_type = "NORMAL"
    transform.convert_from = "WORLD"
    transform.convert_to = "CAMERA"
    scale = nodes.new("ShaderNodeVectorMath")
    scale.operation = "SCALE"
    scale.inputs[3].default_value = 0.5
    offset = nodes.new("ShaderNodeVectorMath")
    offset.operation = "ADD"
    offset.inputs[1].default_value = (0.5, 0.5, 0.5)
    emission = nodes.new("ShaderNodeEmission")
    material.node_tree.links.new(geometry.outputs["Normal"], transform.inputs["Vector"])
    material.node_tree.links.new(transform.outputs["Vector"], scale.inputs[0])
    material.node_tree.links.new(scale.outputs["Vector"], offset.inputs[0])
    material.node_tree.links.new(offset.outputs["Vector"], emission.inputs["Color"])
    material.node_tree.links.new(emission.outputs["Emission"], output.inputs["Surface"])
    return material


def _renormalize_normal_exr(path, scene):
    import bpy

    image = bpy.data.images.load(str(path), check_existing=False)
    try:
        width, height = int(image.size[0]), int(image.size[1])
        rgba = np.empty(width * height * 4, dtype=np.float32)
        image.pixels.foreach_get(rgba)
        rgba = rgba.reshape(height, width, 4)
        mask = rgba[:, :, 3] > 1e-6
        if np.any(mask):
            normals = rgba[:, :, :3][mask] * 2.0 - 1.0
            lengths = np.linalg.norm(normals, axis=1, keepdims=True)
            valid = lengths[:, 0] > 1e-8
            normals[valid] /= lengths[valid]
            rgba[:, :, :3][mask] = normals * 0.5 + 0.5
        rgba[:, :, :3][~mask] = 0.0
        image.pixels.foreach_set(rgba.reshape(-1))
        image.filepath_raw = str(path)
        image.file_format = "OPEN_EXR"
        image.save_render(str(path), scene=scene)
    finally:
        bpy.data.images.remove(image)


def render_normal(scene, camera, target_path):
    import bpy

    target_path = Path(target_path)
    target_path.parent.mkdir(parents=True, exist_ok=True)
    view_layer = scene.view_layers[0]
    image = scene.render.image_settings
    saved = {
        "camera": scene.camera,
        "filepath": scene.render.filepath,
        "engine": scene.render.engine,
        "film_transparent": scene.render.film_transparent,
        "file_format": image.file_format,
        "color_mode": image.color_mode,
        "color_depth": image.color_depth,
        "color_management": image.color_management,
        "override": view_layer.material_override,
        "view_transform": scene.view_settings.view_transform,
        "look": scene.view_settings.look,
        "exposure": scene.view_settings.exposure,
        "gamma": scene.view_settings.gamma,
    }
    material = _normal_material()
    try:
        scene.camera = camera
        if scene.render.engine not in {"CYCLES", "BLENDER_EEVEE", "BLENDER_EEVEE_NEXT"}:
            try:
                scene.render.engine = "BLENDER_EEVEE"
            except TypeError:
                scene.render.engine = "BLENDER_EEVEE_NEXT"
        view_layer.material_override = material
        scene.render.film_transparent = True
        image.file_format = "OPEN_EXR"
        image.color_mode = "RGBA"
        image.color_depth = "32"
        if hasattr(image, "color_management"):
            image.color_management = "OVERRIDE"
        scene.view_settings.view_transform = "Raw"
        scene.view_settings.exposure = 0.0
        scene.view_settings.gamma = 1.0
        scene.render.filepath = str(target_path.with_suffix(""))
        bpy.ops.render.render(write_still=True)
        _renormalize_normal_exr(target_path, scene)
    finally:
        scene.camera = saved["camera"]
        scene.render.filepath = saved["filepath"]
        scene.render.engine = saved["engine"]
        scene.render.film_transparent = saved["film_transparent"]
        image.file_format = saved["file_format"]
        image.color_mode = saved["color_mode"]
        image.color_depth = saved["color_depth"]
        if hasattr(image, "color_management"):
            image.color_management = saved["color_management"]
        view_layer.material_override = saved["override"]
        scene.view_settings.view_transform = saved["view_transform"]
        scene.view_settings.look = saved["look"]
        scene.view_settings.exposure = saved["exposure"]
        scene.view_settings.gamma = saved["gamma"]
        bpy.data.materials.remove(material)


def id_png_to_npy(source_path, target_path, id_map):
    import bpy

    source_path = Path(source_path)
    image = bpy.data.images.load(str(source_path), check_existing=False)
    try:
        width, height = int(image.size[0]), int(image.size[1])
        rgba = np.empty(width * height * 4, dtype=np.float32)
        image.pixels.foreach_get(rgba)
        rgb = rgba.reshape(height, width, 4)[:, :, :3]
        colors = np.asarray([item["color"][:3] for item in id_map.get("items", [])], dtype=np.float32)
        output = np.full((height, width), -1, dtype=np.int32)
        if len(colors):
            flat = rgb.reshape(-1, 3)
            for start in range(0, len(flat), 262144):
                chunk = flat[start:start + 262144]
                distance = ((chunk[:, None, :] - colors[None, :, :]) ** 2).sum(axis=2)
                nearest = np.argmin(distance, axis=1)
                minimum = distance[np.arange(len(chunk)), nearest]
                values = np.where(minimum < 0.04, nearest + 1, -1)
                output.reshape(-1)[start:start + len(chunk)] = values.astype(np.int32)
        output = np.flipud(output)
        target_path = Path(target_path)
        target_path.parent.mkdir(parents=True, exist_ok=True)
        np.save(target_path, output)
        return {"width": width, "height": height, "object_count": int(len(colors))}
    finally:
        bpy.data.images.remove(image)



def _triangle_id_material():
    import bpy

    material = bpy.data.materials.new("GS_MeshGuided_TriangleId")
    material.use_nodes = True
    nodes = material.node_tree.nodes
    nodes.clear()
    output = nodes.new("ShaderNodeOutputMaterial")
    attribute = nodes.new("ShaderNodeAttribute")
    attribute.attribute_name = "gs_triangle_id"
    combine = nodes.new("ShaderNodeCombineColor")
    emission = nodes.new("ShaderNodeEmission")
    material.node_tree.links.new(attribute.outputs["Factor"], combine.inputs["Red"])
    material.node_tree.links.new(combine.outputs["Color"], emission.inputs["Color"])
    material.node_tree.links.new(emission.outputs["Emission"], output.inputs["Surface"])
    return material


class TriangleIdRenderer:
    """Render integer IDs from the exact rows of training_mesh.npz."""

    def __init__(self, scene, training_mesh_path):
        import bpy

        with np.load(training_mesh_path, allow_pickle=False) as data:
            vertices = np.asarray(data["vertices_world"], dtype=np.float32)
            triangles = np.asarray(data["triangles"], dtype=np.int32)
        self.scene = scene
        self.triangle_count = len(triangles)
        self.mesh = bpy.data.meshes.new("GS_MeshGuided_TriangleIdMesh")
        self.mesh.vertices.add(len(vertices))
        self.mesh.vertices.foreach_set("co", vertices.reshape(-1))
        self.mesh.loops.add(self.triangle_count * 3)
        self.mesh.loops.foreach_set("vertex_index", triangles.reshape(-1))
        self.mesh.polygons.add(self.triangle_count)
        self.mesh.polygons.foreach_set("loop_start", np.arange(self.triangle_count, dtype=np.int32) * 3)
        self.mesh.polygons.foreach_set("loop_total", np.full(self.triangle_count, 3, dtype=np.int32))
        attribute = self.mesh.attributes.new(name="gs_triangle_id", type="INT", domain="FACE")
        attribute.data.foreach_set("value", np.arange(1, self.triangle_count + 1, dtype=np.int32))
        self.mesh.update()
        self.material = _triangle_id_material()
        self.mesh.materials.append(self.material)
        self.object = bpy.data.objects.new("GS_MeshGuided_TriangleId", self.mesh)
        scene.collection.objects.link(self.object)
        self.object.hide_render = True

    def render(self, camera, target_path):
        import bpy

        target_path = Path(target_path)
        target_path.parent.mkdir(parents=True, exist_ok=True)
        render_path = target_path.with_suffix(".triangle_id.exr")
        view_layer = self.scene.view_layers[0]
        image_settings = self.scene.render.image_settings
        rendered_image = None
        saved_visibility = {obj: bool(obj.hide_render) for obj in self.scene.objects}
        saved = {
            "camera": self.scene.camera,
            "engine": self.scene.render.engine,
            "film_transparent": self.scene.render.film_transparent,
            "filepath": self.scene.render.filepath,
            "file_format": image_settings.file_format,
            "color_mode": image_settings.color_mode,
            "color_depth": image_settings.color_depth,
            "color_management": getattr(image_settings, "color_management", None),
            "override": view_layer.material_override,
            "view_transform": self.scene.view_settings.view_transform,
            "look": self.scene.view_settings.look,
            "exposure": self.scene.view_settings.exposure,
            "gamma": self.scene.view_settings.gamma,
            "filter_size": self.scene.render.filter_size,
        }
        try:
            for obj in self.scene.objects:
                if obj.type in {"MESH", "CURVE", "SURFACE", "FONT", "META", "CURVES"}:
                    obj.hide_render = obj != self.object
            self.object.hide_render = False
            self.scene.camera = camera
            if self.scene.render.engine not in {"BLENDER_EEVEE", "BLENDER_EEVEE_NEXT"}:
                try:
                    self.scene.render.engine = "BLENDER_EEVEE"
                except TypeError:
                    self.scene.render.engine = "BLENDER_EEVEE_NEXT"
            view_layer.material_override = None
            self.scene.render.film_transparent = True
            self.scene.render.filter_size = 0.01
            image_settings.file_format = "OPEN_EXR"
            image_settings.color_mode = "RGBA"
            image_settings.color_depth = "32"
            if hasattr(image_settings, "color_management"):
                image_settings.color_management = "OVERRIDE"
            self.scene.view_settings.view_transform = "Raw"
            self.scene.view_settings.exposure = 0.0
            self.scene.view_settings.gamma = 1.0
            self.scene.render.filepath = str(render_path.with_suffix(""))
            bpy.ops.render.render(write_still=True)
            if not render_path.is_file():
                raise RuntimeError(f"Triangle ID render did not produce {render_path}")
            rendered_image = bpy.data.images.load(str(render_path), check_existing=False)
            width, height = int(rendered_image.size[0]), int(rendered_image.size[1])
            rgba = np.empty(width * height * 4, dtype=np.float32)
            rendered_image.pixels.foreach_get(rgba)
            rgba = rgba.reshape(height, width, 4)
            values = rgba[:, :, 0]
            rounded = np.rint(values)
            valid = (
                (rgba[:, :, 3] > 0.999)
                & np.isfinite(values)
                & (np.abs(values - rounded) < 0.01)
                & (rounded >= 1)
                & (rounded <= self.triangle_count)
            )
            output = np.full((height, width), -1, dtype=np.int64)
            output[valid] = rounded[valid].astype(np.int64) - 1
            output = np.flipud(output)
            temporary = target_path.with_suffix(target_path.suffix + ".tmp.npy")
            np.save(temporary, output)
            temporary.replace(target_path)
            return {
                "width": width,
                "height": height,
                "valid_pixel_count": int(np.count_nonzero(valid)),
                "triangle_count": int(self.triangle_count),
            }
        finally:
            self.scene.camera = saved["camera"]
            self.scene.render.engine = saved["engine"]
            self.scene.render.film_transparent = saved["film_transparent"]
            self.scene.render.filepath = saved["filepath"]
            self.scene.render.filter_size = saved["filter_size"]
            image_settings.file_format = saved["file_format"]
            image_settings.color_mode = saved["color_mode"]
            image_settings.color_depth = saved["color_depth"]
            if hasattr(image_settings, "color_management"):
                image_settings.color_management = saved["color_management"]
            view_layer.material_override = saved["override"]
            self.scene.view_settings.view_transform = saved["view_transform"]
            self.scene.view_settings.look = saved["look"]
            self.scene.view_settings.exposure = saved["exposure"]
            self.scene.view_settings.gamma = saved["gamma"]
            for obj, hidden in saved_visibility.items():
                if obj.name in bpy.data.objects:
                    obj.hide_render = hidden
            if rendered_image is not None:
                bpy.data.images.remove(rendered_image)
            render_path.unlink(missing_ok=True)

    def close(self):
        import bpy

        if self.object is not None and self.object.name in bpy.data.objects:
            bpy.data.objects.remove(self.object, do_unlink=True)
        if self.material is not None and self.material.name in bpy.data.materials:
            bpy.data.materials.remove(self.material)
        if self.mesh is not None and self.mesh.name in bpy.data.meshes:
            bpy.data.meshes.remove(self.mesh)
        self.object = None
        self.material = None
        self.mesh = None
