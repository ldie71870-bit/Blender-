"""Editable closed mesh volumes for contour route constraints (metres)."""
import math

import bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree


def is_region(obj):
    return bool(obj and (obj.get('gs_path_region') or obj.original.get('gs_path_region')))


class RegionVolume:
    def __init__(self, obj, depsgraph, scale):
        if obj is None or obj.type != 'MESH':
            raise ValueError('请选择封闭 Mesh 区域框')
        if obj.mode == 'EDIT':
            obj.update_from_editmode()
        evaluated = obj.evaluated_get(depsgraph)
        mesh = evaluated.to_mesh()
        bm = bmesh.new()
        try:
            bm.from_mesh(mesh)
            if not bm.faces or any(not e.is_manifold for e in bm.edges) or any(not v.link_faces for v in bm.verts):
                raise ValueError('区域网格必须封闭：请补齐缺面、删除孤立点线后重试')
            bmesh.ops.triangulate(bm, faces=list(bm.faces))
            bm.verts.ensure_lookup_table()
            bm.verts.index_update()
            vertices = [(evaluated.matrix_world @ v.co) * scale for v in bm.verts]
            faces = [tuple(v.index for v in f.verts) for f in bm.faces]
            self.low = Vector(tuple(min(v[j] for v in vertices) for j in range(3)))
            self.high = Vector(tuple(max(v[j] for v in vertices) for j in range(3)))
            if min(self.high-self.low) < 1e-6:
                raise ValueError('区域必须具有三维体积，不能是平面')
            self.bvh = BVHTree.FromPolygons(vertices, faces, all_triangles=True)
            self.limit = len(faces) + 1
            self.name = obj.name
            self.epsilon = max(1e-6, (self.high-self.low).length * 1e-6)
            self.cache = {}
        finally:
            bm.free()
            evaluated.to_mesh_clear()

    def intersections(self, origin, direction, distance):
        origin = Vector(origin)
        direction = Vector(direction).normalized()
        offset = 0.0
        hits = []
        for _ in range(self.limit):
            point, _, _, depth = self.bvh.ray_cast(origin+direction*offset, direction, max(0.,distance-offset))
            if point is None:
                return hits
            offset += depth
            hits.append(offset)
            offset += self.epsilon
            if offset >= distance:
                return hits
        raise ValueError('区域边界交点异常，请检查重叠面或自相交')

    def contains(self, point):
        point = Vector(point)
        if any(point[j] < self.low[j]-self.epsilon or point[j] > self.high[j]+self.epsilon for j in range(3)):
            return False
        key = tuple(point)
        if key in self.cache:
            return self.cache[key]
        nearest = self.bvh.find_nearest(point)
        if nearest[0] is not None and nearest[3] <= self.epsilon:
            return True
        distance = (self.high-self.low).length*2 + 1
        votes = [len(self.intersections(point, d, distance)) % 2 for d in
                 ((1,.371,.529),(-.419,1,.237),(.193,-.617,1))]
        inside = sum(votes) >= 2
        if len(self.cache) > 200000:
            self.cache.clear()
        self.cache[key] = inside
        return inside

    def segment_inside(self, a, b):
        a, b = Vector(a), Vector(b)
        if not self.contains(a) or not self.contains(b):
            return False
        distance = (b-a).length
        if distance <= self.epsilon:
            return True
        direction = (b-a)/distance
        # Check every interval between boundary hits, including narrow concavities.
        cuts = [0.] + self.intersections(a, direction, distance) + [distance]
        return all(self.contains(a+direction*((left+right)*.5))
                   for left,right in zip(cuts,cuts[1:]) if right-left > self.epsilon)

    def sampling_bounds(self, scene_bounds):
        low, high = (p.copy() for p in scene_bounds)
        for j in (0,1):
            low[j] = max(low[j],self.low[j])
            high[j] = min(high[j],self.high[j])
        if any(low[j] >= high[j] for j in (0,1)):
            raise ValueError('区域框与可见场景没有交集')
        # Keep scene Z bounds: supporting floors may lie below the region box.
        return low, high


def active_region(scene, settings, depsgraph=None):
    if not getattr(settings, 'contour_region_enabled', False):
        return None
    obj = settings.contour_region_object
    if obj is None or scene.objects.get(obj.name) is None:
        raise ValueError('请创建或指定当前场景中的区域框')
    import bpy
    return RegionVolume(obj, depsgraph or bpy.context.evaluated_depsgraph_get(),
                        max(1e-9,scene.unit_settings.scale_length))
