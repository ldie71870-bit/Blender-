import numpy as np


SH_C0 = 0.28209479177387814


def _rotation_to_quaternion(matrices):
    """Convert orthonormal 3x3 matrices to normalized WXYZ quaternions."""
    matrices = np.asarray(matrices, dtype=np.float64)
    quaternions = np.empty((len(matrices), 4), dtype=np.float64)
    for index, matrix in enumerate(matrices):
        trace = float(np.trace(matrix))
        if trace > 0.0:
            scale = np.sqrt(trace + 1.0) * 2.0
            q = (0.25 * scale,
                 (matrix[2, 1] - matrix[1, 2]) / scale,
                 (matrix[0, 2] - matrix[2, 0]) / scale,
                 (matrix[1, 0] - matrix[0, 1]) / scale)
        else:
            axis = int(np.argmax(np.diag(matrix)))
            if axis == 0:
                scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
                q = ((matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale,
                     (matrix[0, 1] + matrix[1, 0]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale)
            elif axis == 1:
                scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
                q = ((matrix[0, 2] - matrix[2, 0]) / scale,
                     (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                     (matrix[1, 2] + matrix[2, 1]) / scale)
            else:
                scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
                q = ((matrix[1, 0] - matrix[0, 1]) / scale,
                     (matrix[0, 2] + matrix[2, 0]) / scale,
                     (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale)
        quaternions[index] = q
    quaternions /= np.maximum(np.linalg.norm(quaternions, axis=1, keepdims=True), 1e-12)
    return quaternions.astype(np.float32)


def initialize_gaussians(samples, normal_thickness_ratio=0.05):
    tangent = np.asarray(samples["tangents"], dtype=np.float32)
    bitangent = np.asarray(samples["bitangents"], dtype=np.float32)
    normal = np.asarray(samples["surface_normals"], dtype=np.float32)
    rotations = np.stack((tangent, bitangent, normal), axis=2)
    quaternions = _rotation_to_quaternion(rotations)
    spacing = np.maximum(np.asarray(samples["spacing"], dtype=np.float32), 1e-7)
    planar = spacing * 0.5
    scale_values = np.column_stack((planar, planar, planar * float(normal_thickness_ratio))).astype(np.float32)
    alpha = np.clip(np.asarray(samples["alpha"], dtype=np.float32), 1e-4, 1.0 - 1e-4)
    colors = np.clip(np.asarray(samples["colors"], dtype=np.float32), 0.0, 1.0)
    return {
        "means": np.asarray(samples["positions"], dtype=np.float32),
        "quats": quaternions,
        "scales": scale_values,
        "log_scales": np.log(scale_values),
        "opacities": np.log(alpha / (1.0 - alpha)).astype(np.float32),
        "alpha": alpha,
        "sh0": ((colors - 0.5) / SH_C0).astype(np.float32),
        "shN": np.empty((len(colors), 0), dtype=np.float32),
        "colors": colors,
        "triangle_ids": np.asarray(samples["triangle_ids"], dtype=np.int64),
        "object_ids": np.asarray(samples["object_ids"], dtype=np.int32),
        "material_ids": np.asarray(samples["material_ids"], dtype=np.int32),
        "barycentrics": np.asarray(samples["barycentrics"], dtype=np.float32),
        "surface_normals": normal,
        "original_positions": np.asarray(samples["positions"], dtype=np.float32),
        "normal_offsets": np.zeros(len(colors), dtype=np.float32),
        "uvs": np.asarray(samples["uvs"], dtype=np.float32),
        "roughness": np.asarray(samples["roughness"], dtype=np.float32),
        "metallic": np.asarray(samples["metallic"], dtype=np.float32),
        "emission": np.asarray(samples["emission"], dtype=np.float32),
    }

