"""Camera rays against real scene geometry, excluding route display tubes.

A build owns one lazy BVH snapshot. No hide flags, source geometry, or view
selection are changed. Camera creation cannot invalidate this obstacle mesh.
"""
from bisect import bisect_right
import bpy
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
from . import region_volume

SCOPES = {}


def _path_names(settings):
    names = set()
    if settings is not None:
        collection = getattr(settings, 'path_collection', None)
        if collection:
            names.update(o.name for o in collection.all_objects if o.type == 'CURVE')
        obj = getattr(settings, 'path_object', None)
        if obj and obj.type == 'CURVE': names.add(obj.name)
    return names


def _ignored(obj, paths):
    original = getattr(obj, 'original', obj)
    return bool(original.name in paths or region_volume.is_region(original)
        or original.get('gs_camera_mesh_visual') or original.get('gs_shell_debug_proxy')
        or original.get('gs_contour_version') or original.get('gs_route_role'))


def begin(scene, settings):
    key = scene.as_pointer()
    scope = SCOPES.get(key)
    if scope is None:
        scope = dict(key=key, depth=0, paths=_path_names(settings), query=None)
        SCOPES[key] = scope
    scope['depth'] += 1
    return scope


def end(token):
    if token is None: return
    token['depth'] -= 1
    if token['depth'] <= 0 and SCOPES.get(token['key']) is token:
        SCOPES.pop(token['key'], None)
        token['query'] = None


@bpy.app.handlers.persistent
def shutdown(*args):
    SCOPES.clear()


class CollisionQuery:
    def __init__(self, depsgraph, paths):
        vertices, polygons = [], []
        self.ends, self.owners, self.matrices, self.faces = [], [], [], []
        for instance in depsgraph.object_instances:
            obj = instance.object
            original = obj.original
            if _ignored(obj, paths) or original.type not in {'MESH', 'CURVE', 'SURFACE', 'FONT', 'META'}:
                continue
            if not instance.is_instance and not original.visible_get(): continue
            mesh = obj.to_mesh()
            if mesh is None: continue
            try:
                matrix = instance.matrix_world.copy()
                offset = len(vertices)
                vertices.extend(matrix @ vertex.co for vertex in mesh.vertices)
                mirrored = matrix.to_3x3().determinant() < 0
                for polygon in mesh.polygons:
                    indices = tuple(offset+i for i in polygon.vertices)
                    if len(indices) < 3: continue
                    polygons.append(tuple(reversed(indices)) if mirrored else indices)
                    self.faces.append(polygon.index)
                self.ends.append(len(polygons))
                self.owners.append(original)
                self.matrices.append(matrix)
            finally:
                obj.to_mesh_clear()
        self.bvh = BVHTree.FromPolygons(vertices, polygons) if polygons else None

    def cast(self, origin, direction, distance):
        origin, direction = Vector(origin), Vector(direction)
        if direction.length_squared < 1e-20 or distance <= 0 or self.bvh is None:
            return False, origin, Vector(), -1, None, Matrix.Identity(4)
        direction.normalize()
        point, normal, face, depth = self.bvh.ray_cast(origin, direction, float(distance))
        if point is None:
            return False, origin+direction*distance, Vector(), -1, None, Matrix.Identity(4)
        owner = bisect_right(self.ends, face)
        return True, point, normal, self.faces[face], self.owners[owner], self.matrices[owner]


def ray_cast(scene, depsgraph, origin, direction, distance, settings=None):
    scope = SCOPES.get(scene.as_pointer())
    if scope is not None:
        if scope['query'] is None: scope['query'] = CollisionQuery(depsgraph, scope['paths'])
        return scope['query'].cast(origin, direction, distance)
    # Standalone callers retain Blender's native hit semantics unless a display
    # is actually encountered. The fallback excludes all displays in one pass,
    # rather than repeatedly hitting the same tube at 10-micron offsets.
    origin, direction = Vector(origin), Vector(direction)
    if direction.length_squared < 1e-20 or distance <= 0:
        return False, origin, Vector(), -1, None, Matrix.Identity(4)
    direction.normalize()
    hit = scene.ray_cast(depsgraph, origin, direction, distance=float(distance))
    paths = _path_names(settings or getattr(scene, 'gs_colmap_settings', None))
    if not hit[0] or hit[4] is None or not _ignored(hit[4], paths): return hit
    return CollisionQuery(depsgraph, paths).cast(origin, direction, distance)
