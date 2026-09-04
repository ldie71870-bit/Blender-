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


def select_lines(targets, observations, candidates, budget, baseline, angle):
    """Accept only lines that finish real missing two-view constraints."""
    observations={key:list(value) for key,value in observations.items()}
    covered={i for i,t in enumerate(targets) if adequate(t['point'],observations.get(i,()),baseline,angle)}
    before=set(covered)
    remaining=list(candidates)
    selected=[]
    while remaining and len(selected)<budget:
        best=None
        for index,candidate in enumerate(remaining):
            gains=[]
            for key,views in candidate['observations'].items():
                if key not in covered and adequate(targets[key]['point'],observations.get(key,[])+views,baseline,angle):
                    gains.append(key)
            weight=sum(targets[key].get('weight',1.) for key in gains)
            score=weight/(.5+.3*candidate['length'])
            if gains and (best is None or score>best[0]):
                best=(score,index,gains)
        if best is None:break
        _,index,gains=best
        candidate=remaining.pop(index)
        candidate['gain']=len(gains)
        candidate['targets']=gains
        selected.append(candidate)
        for key,views in candidate['observations'].items():
            observations.setdefault(key,[]).extend(views)
        covered.update(gains)
    return selected,before,covered
