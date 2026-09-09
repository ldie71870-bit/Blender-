"""Continuous surface strokes independent of heatmap sampling density."""
import math
from bisect import bisect_right
from mathutils import Vector
from mathutils.kdtree import KDTree
from mathutils.bvhtree import BVHTree


def _visible(bvh,origin,p):
    d=p-origin
    if d.length<1e-7:return False
    hit,_,_,distance=bvh.ray_cast(origin,d.normalized(),d.length+.002)
    return hit is not None and abs(distance-d.length)<=.003


def paint(st,origin,direction,radius,erase=False,hit_info=None):
    bvh,ends,owners=st['probe'].geometry
    hit,n,face,_=hit_info if hit_info is not None else bvh.ray_cast(origin,direction)
    if hit is None:st.pop('brush_last_dab',None);return 0
    last=st.get('brush_last_dab')
    if last and last[1]==erase and last[2]==radius and (hit-last[0]).length<min(radius*.18,.05):return 0
    st['brush_last_dab']=(hit.copy(),erase,radius)
    patches=st.setdefault('brush_patches',{});changes=set()
    if erase:
        for i in tuple(st['selected']):
            target=patches.get(i)
            if target is None:continue
            if (target['p']-hit).length<=radius and _visible(bvh,origin,target['p']):
                st['selected'].remove(i);changes.add(i)
        # Legacy / already materialized selections can also be erased.
        for _,i,_ in st['tree'].find_range(hit,radius):
            if i in st['selected'] and _visible(bvh,origin,st['samples'][i]['p']):st['selected'].remove(i);changes.add(i)
    else:
        axis=Vector((0,0,1)) if abs(n.z)<.9 else Vector((1,0,0))
        u=n.cross(axis).normalized();v=n.cross(u).normalized()
        owner=owners[bisect_right(ends,face)].name if owners else ''
        # Concentric ray-projected surface cells, not mesh vertices or old coverage cells.
        verts=[(hit,n,face)]
        rings=3;segments=16
        for r in range(1,rings+1):
            for j in range(segments):
                q=hit+(u*math.cos(j*math.tau/segments)+v*math.sin(j*math.tau/segments))*(radius*r/rings)
                delta=q-origin
                p,pn,pface,_=bvh.ray_cast(origin,delta.normalized(),delta.length+radius)
                if p is None or (p-hit).length>radius*1.05:verts.append(None)
                else:verts.append((p,pn,pface))
        tris=[(0,1+j,1+(j+1)%segments) for j in range(segments)]
        for r in range(1,rings):
            a=1+(r-1)*segments;b=1+r*segments
            for j in range(segments):
                k=(j+1)%segments;tris.extend(((a+j,b+j,b+k),(a+j,b+k,a+k)))
        lookup=st.setdefault('brush_patch_lookup',{})
        def add_target(p,pn,pface,points):
            owner_name=owners[bisect_right(ends,pface)].name if owners else owner
            key=(owner_name,*(round(x/.005) for x in p),*(round(x*5) for x in pn))
            idx=lookup.get(key)
            if idx is None:
                idx=st.get('brush_next_id',-1);st['brush_next_id']=idx-1;lookup[key]=idx
                st.setdefault('brush_group_members',{}).setdefault((-idx-1)//128,set()).add(idx)
                patches[idx]=dict(p=p.copy(),n=pn.copy(),projection_normal=pn.copy(),owner=owner_name,triangles=points,batch=None,origin=origin.copy())
            if idx not in st['selected']:st['selected'].add(idx);changes.add(idx)
        any_center=False
        for tri in tris:
            ps=[verts[i] for i in tri]
            if any(p is None for p in ps):continue
            # A projected cell must stay on one object instance/surface sheet.
            if len({bisect_right(ends,p[2]) for p in ps})!=1:continue
            if any(ps[0][1].dot(p[1])<.3 for p in ps[1:]):continue
            if max((ps[i][0]-ps[(i+1)%3][0]).length for i in range(3))>radius*.65:continue
            p=sum((v[0] for v in ps),Vector())/3
            delta=p-origin
            nearest,pn,pface,_=bvh.ray_cast(origin,delta.normalized(),delta.length+radius*.12)
            if nearest is None or (nearest-p).length>radius*.12:continue
            if bisect_right(ends,pface)!=bisect_right(ends,ps[0][2]) or pn.dot(ps[0][1])<.3:continue
            points=[tuple((v[0]+v[1]*.002)/st['scale']) for v in ps]
            add_target(nearest,pn,pface,points)
            if 0 in tri:any_center=True
        # Tiny triangles/slivers still acquire a target at the exact cursor hit.
        if not any_center:
            small=[]
            for divisor in (12,48,192):
                small=[]
                for j in range(3):
                    q=hit+(u*math.cos(j*math.tau/3)+v*math.sin(j*math.tau/3))*(radius/divisor)
                    delta=q-origin;p,pn,pface,_=bvh.ray_cast(origin,delta.normalized(),delta.length+radius/divisor)
                    if p is None or pface!=face:small=[];break
                    small.append(tuple((p+pn*.002)/st['scale']))
                if small:break
            add_target(hit,n,face,small)
    invalidate(st,changes)
    if 'sample_chunks' in st:
        for i in changes:st.setdefault('selection_dirty',set()).update(st['sample_chunks'].get(i,()))
    return len(changes)


def invalidate(st,ids):
    dirty=st.setdefault('brush_dirty_groups',set())
    for i in ids:
        if i<0:dirty.add((-i-1)//128)


def draw(st,shader):
    import gpu
    from gpu_extras.batch import batch_for_shader
    batches=st.setdefault('brush_gpu_groups',{})
    for group in st.get('brush_dirty_groups',()):
        active=[st['brush_patches'][i] for i in st.get('brush_group_members',{}).get(group,()) if i in st['selected'] and i in st.get('brush_patches',{})]
        points=[p for item in active for p in item['triangles']]
        tiny=[tuple((item['p']+item['n']*.003)/st['scale']) for item in active if not item['triangles']]
        if points or tiny:
            batches[group]=(batch_for_shader(shader,'TRIS',{'pos':points,'color':[(.1,.6,1.,.78)]*len(points)}) if points else None,
                            batch_for_shader(shader,'POINTS',{'pos':tiny,'color':[(.1,.6,1.,1.)]*len(tiny)}) if tiny else None)
        else:batches.pop(group,None)
    st.setdefault('brush_dirty_groups',set()).clear()
    for triangles,points in batches.values():
        if triangles:triangles.draw(shader)
        if points:
            gpu.state.point_size_set(5.);points.draw(shader);gpu.state.point_size_set(1.)


def displayed_target(st,target,index):
    """Read the same triangle interpolation the user painted, not a nearby KD point."""
    origin=target.get('origin')
    if origin is None or index is None:return
    delta=target['p']-origin
    if delta.length<1e-7:return
    point,_,face,_=index.ray_cast(origin,delta.normalized(),delta.length+.01)
    if point is None or (point-target['p']).length>.01:return
    tri=st['triangles'][face];vertices=[st['vertices'][v] for v in tri]
    if any(st['samples'][v[2]]['owner']!=target['owner'] for v in vertices):return
    a,b,c=[p+n*.001 for p,n,_ in vertices]
    ab,ac,ap=b-a,c-a,point-a
    aa,bb,cc=ab.dot(ab),ab.dot(ac),ac.dot(ac)
    denominator=aa*cc-bb*bb
    if abs(denominator)<1e-20:return
    v=(cc*ap.dot(ab)-bb*ap.dot(ac))/denominator
    w=(aa*ap.dot(ac)-bb*ap.dot(ab))/denominator
    weights=[max(0.,min(1.,t)) for t in (1-v-w,v,w)]
    total=sum(weights)
    if total<=0:return
    weights=[t/total for t in weights]
    score=sum(weight*max(0.,min(1.,st['samples'][vertex[2]]['score'])) for weight,vertex in zip(weights,vertices))
    # Display scores are diagnostic. Do not replace a physical hit normal with
    # an interpolated/old normal that may be the reason the heatmap is wrong.
    target['display_score']=score
    target['display_weak']=score<.999


def reconcile_display(st):
    """Index the cached overlay once; no full-scene camera coverage evaluation."""
    targets=[st.get('brush_patches',{}).get(i) for i in st['selected']]
    targets=[target for target in targets if target and 'display_score' not in target]
    if not targets or not st['triangles']:return
    key=(len(st['vertices']),len(st['triangles']))
    cached=st.get('brush_display_index')
    if cached is None or cached[0]!=key:
        yield .005,'关联刷选面与已显示覆盖颜色（不重算全场景）'
        index=BVHTree.FromPolygons([p+n*.001 for p,n,_ in st['vertices']],st['triangles'],all_triangles=True)
        st['brush_display_index']=(key,index)
    else:index=cached[1]
    for i,target in enumerate(targets):
        displayed_target(st,target,index)
        if i%128==0:yield .005+.005*i/len(targets),f'关联刷选覆盖面：{i}/{len(targets)}'


def materialize(st):
    """Insert only painted surfaces into the analysis, once per repair/update."""
    mapping={};samples=st['samples'];vertices=st['vertices']
    for idx in sorted(st['selected']):
        target=st.get('brush_patches',{}).get(idx)
        if target is None:continue
        new=len(samples);mapping[idx]=new
        support=[];origin=target.get('origin');bvh,ends,owners=st['probe'].geometry
        if origin is not None:
            # Interior probes avoid triangle vertices/seams. Compute only when
            # entering analysis, never while moving the brush.
            for corner in target['triangles'][:3]:
                corner=Vector(corner)*st['scale']-target.get('projection_normal',target['n'])*.002
                q=target['p']*.6+corner*.4;delta=q-origin
                p,n,face,_=bvh.ray_cast(origin,delta.normalized(),delta.length+.003)
                if p is None or (p-q).length>.003:continue
                owner=owners[bisect_right(ends,face)].name if owners else ''
                if owner!=target['owner'] or abs(n.dot(target['n']))<.8:continue
                support.append((p,n))
        samples.append(dict(p=target['p'],n=target['n'],owner=target['owner'],views=[],score=0.,brush_pending=True,
                            support=support,brush_display_score=target.get('display_score'),brush_display_weak=target.get('display_weak',False)))
        points=target['triangles']
        if points:
            base=len(vertices)
            vertices.extend((Vector(p)*st['scale']-target['n']*.002,target['n'],new) for p in points)
            st['triangles'].append(tuple(range(base,base+3)))
    if not mapping:return
    st['selected']={mapping.get(i,i) for i in st['selected']}
    st['brush_history']=[({mapping.get(i,i) for i in add},{mapping.get(i,i) for i in remove}) for add,remove in st.get('brush_history',[])]
    st.pop('stroke_before',None);st.pop('brush_last_dab',None)
    invalidate(st,mapping)
    for i in mapping:st['brush_patches'].pop(i,None)
    st['brush_patch_lookup']={};st['brush_active_dirty']=True
    st.pop('brush_display_index',None)
    st['tree']=KDTree(len(samples))
    for i,s in enumerate(samples):st['tree'].insert(s['p'],i)
    st['tree'].balance();st['batch']=None
    for key in ('sample_chunks','selection_chunks','selection_batches','selection_dirty'):st.pop(key,None)


def observe_pending(heat,st,settings):
    yield from reconcile_display(st)
    materialize(st)
    pending=[s for i,s in enumerate(st['samples']) if s.get('brush_pending') or
             (i in st['selected'] and s.get('coverage_model',1)<heat.OBSERVATION_MODEL)]
    tree=KDTree(len(st['records']))
    for i,r in enumerate(st['records']):tree.insert(r[0],i)
    tree.balance()
    for i,s in enumerate(pending):
        records=[st['records'][j] for _,j,_ in tree.find_range(s['p'],settings.coverage_max_distance)]
        s['views']=heat.sample_views(st,s,records,settings)
        s['score']=len(heat.independent(s['p'],s['views'],settings))/settings.coverage_good_views
        s['coverage_model']=heat.OBSERVATION_MODEL
        s.pop('brush_pending',None)
        if i%32==0:yield .01+.08*i/max(1,len(pending)),f'分析实际刷选表面：{i}/{len(pending)}'
    if pending:
        st['batch']=None;st.pop('disk_saved_key',None)
