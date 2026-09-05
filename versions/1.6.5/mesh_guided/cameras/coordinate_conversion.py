"""Centralized Blender camera to OpenCV coordinate conversion."""

import numpy as np


BLENDER_CAMERA_TO_OPENCV = np.diag([1.0, -1.0, -1.0, 1.0])


def camera_to_world_opencv(camera_to_world_blender):
    matrix = np.asarray(camera_to_world_blender, dtype=np.float64).reshape(4, 4)
    return matrix @ BLENDER_CAMERA_TO_OPENCV


def world_to_camera_opencv(camera_to_world_blender):
    return np.linalg.inv(camera_to_world_opencv(camera_to_world_blender))


def matrices_are_inverse(camera_to_world, world_to_camera, atol=1e-7):
    camera_to_world = np.asarray(camera_to_world, dtype=np.float64).reshape(4, 4)
    world_to_camera = np.asarray(world_to_camera, dtype=np.float64).reshape(4, 4)
    return bool(np.allclose(camera_to_world @ world_to_camera, np.eye(4), atol=atol))


def project_world_points(points, world_to_camera, fx, fy, cx, cy):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    homogeneous = np.concatenate((points, np.ones((len(points), 1))), axis=1)
    camera = (np.asarray(world_to_camera, dtype=np.float64) @ homogeneous.T).T[:, :3]
    z = camera[:, 2]
    valid = z > 1e-9
    pixels = np.full((len(points), 2), np.nan, dtype=np.float64)
    pixels[valid, 0] = fx * camera[valid, 0] / z[valid] + cx
    pixels[valid, 1] = fy * camera[valid, 1] / z[valid] + cy
    return pixels, valid

