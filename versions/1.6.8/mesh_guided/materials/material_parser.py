import numpy as np


def _principled(material):
    if material is None or not material.use_nodes or material.node_tree is None:
        return None
    for node in material.node_tree.nodes:
        if node.type == "BSDF_PRINCIPLED":
            return node
    return None


def _input(node, *names):
    if node is None:
        return None
    for name in names:
        socket = node.inputs.get(name)
        if socket is not None:
            return socket
    return None


def _linked_image(socket, visited=None):
    if socket is None or not socket.is_linked:
        return None
    visited = visited or set()
    for link in socket.links:
        node = link.from_node
        if node.as_pointer() in visited:
            continue
        visited.add(node.as_pointer())
        if node.type == "TEX_IMAGE" and node.image is not None:
            return node.image
        for item in node.inputs:
            image = _linked_image(item, visited)
            if image is not None:
                return image
    return None


def _socket_value(socket, default):
    if socket is None:
        return default
    value = socket.default_value
    if hasattr(value, "__len__"):
        return [float(item) for item in value]
    return float(value)


def describe_material(material):
    node = _principled(material)
    base = _input(node, "Base Color")
    alpha = _input(node, "Alpha")
    roughness = _input(node, "Roughness")
    metallic = _input(node, "Metallic")
    emission = _input(node, "Emission Color", "Emission")
    emission_strength = _input(node, "Emission Strength")
    image = _linked_image(base)
    diffuse = list(getattr(material, "diffuse_color", (0.8, 0.8, 0.8, 1.0))) if material else [0.8] * 3 + [1.0]
    emission_value = np.asarray(
        _socket_value(emission, [0.0, 0.0, 0.0, 1.0])[:3], dtype=np.float32,
    ) * float(_socket_value(emission_strength, 0.0))
    return {
        "name": material.name if material else "",
        "base_color": _socket_value(base, diffuse)[:4],
        "alpha": float(_socket_value(alpha, diffuse[3] if len(diffuse) > 3 else 1.0)),
        "roughness": float(_socket_value(roughness, 0.5)),
        "metallic": float(_socket_value(metallic, 0.0)),
        "emission": emission_value.tolist(),
        "emission_strength": float(_socket_value(emission_strength, 0.0)),
        "base_color_image": image.name if image else "",
        "image_source": image.source if image else "",
        "image_filepath": image.filepath if image else "",
        "image_packed": bool(image and image.packed_file),
        "image_has_data": bool(image and image.has_data),
        "procedural_base_color": bool(base and base.is_linked and image is None),
    }


def describe_materials(names):
    import bpy
    return [describe_material(bpy.data.materials.get(name)) for name in names]


class MaterialSampler:
    def __init__(self, material_names):
        import bpy
        self.materials = [describe_material(bpy.data.materials.get(name)) for name in material_names]
        self._images = {}

    def _image_pixels(self, name):
        if not name:
            return None
        if name in self._images:
            return self._images[name]
        import bpy
        image = bpy.data.images.get(name)
        if image is None or not image.has_data or image.size[0] <= 0 or image.size[1] <= 0:
            self._images[name] = None
            return None
        width, height = int(image.size[0]), int(image.size[1])
        pixels = np.empty(width * height * 4, dtype=np.float32)
        image.pixels.foreach_get(pixels)
        value = (pixels.reshape(height, width, 4), width, height)
        self._images[name] = value
        return value

    @staticmethod
    def _bilinear(image, uv):
        pixels, width, height = image
        uv = np.asarray(uv, dtype=np.float32)
        x = np.mod(uv[:, 0], 1.0) * max(1, width - 1)
        y = np.mod(uv[:, 1], 1.0) * max(1, height - 1)
        x0 = np.floor(x).astype(np.int32)
        y0 = np.floor(y).astype(np.int32)
        x1 = np.minimum(x0 + 1, width - 1)
        y1 = np.minimum(y0 + 1, height - 1)
        tx = (x - x0)[:, None]
        ty = (y - y0)[:, None]
        a = pixels[y0, x0] * (1.0 - tx) + pixels[y0, x1] * tx
        b = pixels[y1, x0] * (1.0 - tx) + pixels[y1, x1] * tx
        return a * (1.0 - ty) + b * ty

    def sample(self, material_ids, uvs):
        material_ids = np.asarray(material_ids, dtype=np.int32)
        count = len(material_ids)
        colors = np.full((count, 3), 0.8, dtype=np.float32)
        alpha = np.ones(count, dtype=np.float32)
        roughness = np.full(count, 0.5, dtype=np.float32)
        metallic = np.zeros(count, dtype=np.float32)
        emission = np.zeros((count, 3), dtype=np.float32)
        for material_id in np.unique(material_ids):
            mask = material_ids == material_id
            if material_id < 0 or material_id >= len(self.materials):
                continue
            info = self.materials[int(material_id)]
            base = np.asarray(info["base_color"], dtype=np.float32)
            colors[mask] = base[:3]
            alpha[mask] = float(info["alpha"] * (base[3] if len(base) > 3 else 1.0))
            roughness[mask] = float(info["roughness"])
            metallic[mask] = float(info["metallic"])
            emission[mask] = np.asarray(info["emission"], dtype=np.float32)
            image = self._image_pixels(info["base_color_image"])
            if image is not None:
                sampled = self._bilinear(image, np.asarray(uvs)[mask])
                colors[mask] = sampled[:, :3]
                alpha[mask] *= sampled[:, 3]
        return {
            "colors": np.clip(colors, 0.0, 1.0),
            "alpha": np.clip(alpha, 1e-4, 1.0 - 1e-4),
            "roughness": np.clip(roughness, 0.0, 1.0),
            "metallic": np.clip(metallic, 0.0, 1.0),
            "emission": np.maximum(emission, 0.0),
        }

