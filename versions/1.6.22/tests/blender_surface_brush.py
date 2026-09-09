"""Large sampled surface: local cache reuse, brush occlusion and curved cursor."""
import bpy,sys,importlib.util,json,time,types,statistics
from pathlib import Path
from mathutils import Vector
from mathutils.kdtree import KDTree
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('brush_perf_addon',ROOT/'__init__.py',submodule_search_locations=[str(ROOT)])
a=importlib.util.module_from_spec(spec);sys.modules[spec.name]=a;spec.loader.exec_module(a)
h=a.coverage_heat

def fixture(n=421):
    step=.02;normal=Vector((0,0,1));points=[Vector((i*step,j*step,0)) for j in range(n) for i in range(n)]
    samples=[dict(p=p,n=normal,score=.5) for p in points]
    tree=KDTree(len(samples))
    for i,p in enumerate(points):tree.insert(p,i)
    tree.balance();size=(n-1)*step
    bvh=BVHTree.FromPolygons([(0,0,0),(size,0,0),(size,size,0),(0,size,0)],[(0,1,2,3)])
    tris=[]
    for y in range(n-1):
        for x in range(n-1):
            i=y*n+x;tris.extend(((i,i+1,i+n),(i+1,i+n+1,i+n)))
    return dict(samples=samples,vertices=[(p,normal,i) for i,p in enumerate(points)],triangles=tris,tree=tree,
                scale=1.,probe=types.SimpleNamespace(geometry=(bvh,[],[])),selected=set(),batch=object(),dirty=False)

st=fixture();base=st['batch'];started=time.perf_counter();h.prepare_selection(st);prep=time.perf_counter()-started
origin=Vector((4,4,2));direction=Vector((0,0,-1));radius=.35
h.surface_cursor(st,origin,direction,radius)
assert len(st['cursor']['lines'])==52
for xyz in st['cursor']['lines']:
    p=Vector(xyz);assert abs(p.z-.004)<.003
original=h.surface_audit.visible;calls=[]
def counting(*args):calls.append(1);return original(*args)
h.surface_audit.visible=counting
h.begin_stroke(st);first=h.paint(st,origin,direction,radius);h.finish_stroke(st)
assert first>60 and st['batch'] is base
assert len(st['selection_dirty'])<len(st['selection_chunks'])/10
first_rays=len(calls);calls.clear();times=[]
for _ in range(80):
    t=time.perf_counter();assert h.paint(st,origin,direction,radius)==0;times.append((time.perf_counter()-t)*1000)
assert len(calls)==0,'Already-selected points must not repeat occlusion rays'
# Moving paint and erasure mark only local chunks and leave the base batch untouched.
h.begin_stroke(st);h.paint(st,Vector((4.3,4,2)),direction,radius);h.finish_stroke(st)
selection=set(st['selected']);h.begin_stroke(st);h.paint(st,origin,direction,radius,erase=True);h.finish_stroke(st)
assert st['selected']!=selection and h.undo_stroke(st) and st['selected']==selection and st['batch'] is base
h.surface_audit.visible=original
# A real curved mesh: every rim vertex lies at the normal offset from the sphere BVH.
bpy.ops.mesh.primitive_uv_sphere_add(segments=96,ring_count=48,radius=1,location=(0,0,0))
obj=bpy.context.object;mesh=obj.data
sphere=BVHTree.FromPolygons([v.co.copy() for v in mesh.vertices],[tuple(p.vertices) for p in mesh.polygons])
curved=dict(scale=1.,probe=types.SimpleNamespace(geometry=(sphere,[],[])))
h.surface_cursor(curved,Vector((0,-3,0)),Vector((0,1,0)),.4)
assert len(curved['cursor']['lines'])>=48
for p in curved['cursor']['lines']:
    nearest=sphere.find_nearest(Vector(p));assert abs(nearest[3]-.004)<.001
h.surface_cursor(curved,Vector((0,-3,0)),Vector((0,-1,0)),.4);assert curved['cursor'] is None
report=dict(samples=len(st['samples']),triangles=len(st['triangles']),total_chunks=len(st['selection_chunks']),local_dirty_chunks=len(st['selection_dirty']),
            first_dab_occlusion_rays=first_rays,repeated_dab_occlusion_rays=0,
            repeated_dab_median_ms=statistics.median(times),one_time_chunk_setup_s=prep,
            base_batch_preserved=True,curved_rim_projection=True,undo_keeps_cache=True)
out=ROOT.parent/'validation_1.6.21';out.mkdir(exist_ok=True);(out/'brush_performance.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print('SURFACE_BRUSH_PERF_OK',json.dumps(report),flush=True)
