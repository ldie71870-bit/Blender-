"""Straight, collision-checked probes into depth openings along retained routes."""
import math
from mathutils import Vector
from . import contour_planner as planner
from .detail_coverage import resample, kd


def discover(routes, probe, budget=24, depth=1.2, progress=None):
    if budget<=0:return [],{'candidates':0,'accepted':0}
    samples=[]
    for layer,label,route in routes:
        for p in resample(route.points,.55):samples.append((layer,label,Vector(p)))
    if not samples:return [],{'candidates':0,'accepted':0}
    bounds=[(min(p[j] for _,_,p in samples),max(p[j] for _,_,p in samples)) for j in range(3)]
    tree=kd([p for _,_,p in samples]); candidates=[]; stats={"samples":len(samples),"openings":0,"unsafe":0,"near_existing":0,"clear_origins":0}
    for sample_index,(layer,label,p) in enumerate(samples):
        stats["clear_origins"]+=int(probe.point_clear(p))
        ring=[]
        for i in range(36):
            angle=i*math.tau/36;direction=Vector((math.cos(angle),math.sin(angle),0))
            hit=probe.cast(p,direction,3.)
            distance=(Vector(hit[0])-p).length if hit else 3.
            ring.append((direction,distance,hit))
        for i,(direction,distance,hit) in enumerate(ring):
            left,right=ring[(i-2)%36][1],ring[(i+2)%36][1]
            jump=distance-min(left,right)
            if jump<.5 or distance<.75:continue
            # Rays just inside a doorway see a far wall; the neighboring rays hit
            # the jamb. Keep the far ray, then advance toward it along a straight
            # segment instead of fitting a contour around the foreground wall.
            stats["openings"]+=1
            reach=min(float(depth),distance-max(.22,probe.margin*1.5))
            end=None
            for fraction in (1.,.8,.6,.4):
                q=p+direction*(reach*fraction)
                if (q-p).length<.4:continue
                if not all(bounds[j][0]-.05<=q[j]<=bounds[j][1]+.05 for j in range(3)):continue
                if probe.segment_clear(p,q):end=q;break
            if end is None:
                stats["unsafe"]+=1;continue
            if tree.find(end)[2]<.24:
                stats["near_existing"]+=1;continue
            score=jump+min(left,right)*.1+(1. if max(left,right)<distance-.35 else 0.)
            candidates.append((score,layer,label,p.copy(),end.copy(),hit))
        if progress and sample_index%20==0:progress('搜索门洞直线探入路线',sample_index,len(samples))
    candidates.sort(key=lambda x:(-x[0],x[1],tuple(x[4])))
    selected=[]; endpoints=[]
    for score,layer,label,p,end,hit in candidates:
        if len(selected)>=budget:break
        if any((end-other).length<.55 for other in endpoints):continue
        route=planner.Route([tuple(p),tuple(end)],kind='DETAIL')
        route.peek_route=True;route.detail_gain=0;route.gap_gain=0
        route.target_objects=[hit[2].name] if hit else []
        route.peek_target=tuple(hit[0]) if hit else tuple(end+(end-p).normalized()*.5)
        selected.append((layer,label,route));endpoints.append(end)
    return selected,{'diagnostics':stats,'candidates':len(candidates),'accepted':len(selected),'length_m':sum(planner.length(r.points) for _,_,r in selected),
                     'paths':[{'layer':l,'points':r.points,'target':r.peek_target} for _,l,r in selected]}
