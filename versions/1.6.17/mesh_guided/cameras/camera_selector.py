import math


COLLECTION_NAME = "GS_MeshGuided_Cameras"


def _camera_distance(a, b):
    return (a.matrix_world.translation - b.matrix_world.translation).length


def select_diverse_cameras(cameras, maximum, position_threshold=0.02, angle_threshold_degrees=2.0):
    """Deterministic pose de-duplication used before the visibility-analysis extension point."""
    from mathutils import Vector

    selected = []
    cosine_threshold = math.cos(math.radians(float(angle_threshold_degrees)))
    for camera in sorted(cameras, key=lambda item: item.name):
        duplicate = False
        current_forward = camera.matrix_world.to_quaternion() @ Vector((0, 0, -1))
        for known in selected:
            if _camera_distance(camera, known) > position_threshold:
                continue
            known_forward = known.matrix_world.to_quaternion() @ Vector((0, 0, -1))
            if current_forward.normalized().dot(known_forward.normalized()) >= cosine_threshold:
                duplicate = True
                break
        if not duplicate:
            selected.append(camera)
        if len(selected) >= maximum:
            break
    return selected


def prepare_mesh_guided_cameras(scene, candidates, maximum=200, deduplicate=True):
    import bpy

    collection = bpy.data.collections.get(COLLECTION_NAME)
    if collection is None:
        collection = bpy.data.collections.new(COLLECTION_NAME)
        scene.collection.children.link(collection)
    for obj in list(collection.objects):
        if obj.get("gs_mesh_guided_camera"):
            bpy.data.objects.remove(obj, do_unlink=True)
    maximum = max(1, int(maximum))
    if deduplicate:
        selected = select_diverse_cameras(candidates, maximum)
    else:
        selected = sorted(candidates, key=lambda item: item.name)[:maximum]
    duplicates = []
    for index, source in enumerate(selected, 1):
        clone = source.copy()
        clone.data = source.data.copy()
        clone.name = f"GS_MeshGuided_Camera_{index:06d}"
        clone.data.name = clone.name + "_Data"
        clone["gs_mesh_guided_camera"] = True
        clone["gs_source_camera"] = source.name
        collection.objects.link(clone)
        duplicates.append(clone)
    return collection, duplicates

