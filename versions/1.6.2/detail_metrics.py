"""Visibility/parallax metrics and marginal-gain selection for detail lines."""
import math


def adequate(target, origins, baseline=.12, angle_degrees=12):
    limit=math.cos(math.radians(angle_degrees))
    for i,a in enumerate(origins):
        av=tuple(a[j]-target[j] for j in range(3))
        al=math.sqrt(sum(v*v for v in av))
        if al<1e-6:continue
        for b in origins[i+1:]:
            if math.dist(a,b)<baseline:continue
            bv=tuple(b[j]-target[j] for j in range(3))
            bl=math.sqrt(sum(v*v for v in bv))
            if bl>1e-6 and sum(av[j]*bv[j] for j in range(3))/(al*bl)<=limit:
                return True
    return False


def select_lines(targets, observations, candidates, budget, baseline, angle, balanced=False,reserve_gaps=0):
    """Accept only lines that finish real missing two-view constraints."""
    observations={key:list(value) for key,value in observations.items()}
    covered={i for i,t in enumerate(targets) if adequate(t['point'],observations.get(i,()),baseline,angle)}
    before=set(covered)
    remaining=list(candidates)
    selected=[]
    counts={c.get('model',0):0 for c in candidates}
    gap_counts={m:0 for m in counts};space_covered=set()
    while remaining and len(selected)<budget:
        best=None
        options=[]
        for index,candidate in enumerate(remaining):
            gains=[]
            for key,views in candidate['observations'].items():
                if key not in covered and adequate(targets[key]['point'],observations.get(key,[])+views,baseline,angle):
                    gains.append(key)
            weight=sum(targets[key].get('weight',1.) for key in gains)
            space_gain=len(candidate.get('space_targets',set())-space_covered)
            weight+=space_gain*.5
            score=weight/(.5+.3*candidate['length'])
            if gains or space_gain:options.append((score,index,gains))
        if balanced and options:
            least=min(counts[remaining[i].get('model',0)] for _,i,_ in options)
            options=[v for v in options if counts[remaining[v[1]].get('model',0)]==least]
        gap_options=[v for v in options if gap_counts[remaining[v[1]].get('model',0)]<reserve_gaps
                     and len(remaining[v[1]].get('space_targets',set())-space_covered)>=5]
        if gap_options:options=gap_options
        if options:best=max(options,key=lambda v:(v[0],-v[1]))
        if best is None:break
        _,index,gains=best
        candidate=remaining.pop(index)
        counts[candidate.get('model',0)]+=1
        new_space=candidate.get('space_targets',set())-space_covered
        candidate['space_gain']=len(new_space);space_covered.update(new_space)
        if new_space:gap_counts[candidate.get('model',0)]+=1
        candidate['gain']=len(gains)
        candidate['targets']=gains
        selected.append(candidate)
        for key,views in candidate['observations'].items():
            observations.setdefault(key,[]).extend(views)
        covered.update(gains)
    return selected,before,covered
