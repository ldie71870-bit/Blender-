"""Station allocation obeys explicit user budgets; never changes route geometry."""
import math


def apportion(total, weights):
    total = max(0, int(total))
    if not weights: return []
    weights = [max(0., float(w)) for w in weights]
    denominator = sum(weights)
    if denominator <= 1e-9: weights = [1.] * len(weights); denominator = len(weights)
    exact = [total * w / denominator for w in weights]
    result = [int(v) for v in exact]
    ranked = sorted(range(len(weights)), key=lambda i: (exact[i] - result[i], weights[i], -i), reverse=True)
    for i in ranked[:total - sum(result)]: result[i] += 1
    return result


def bounded_detail_counts(base, components):
    total = sum(base)
    minimums = [max(1, int(c['object'].get('gs_detail_min_samples', 1))) for c in components]
    if not any(v > 1 for v in minimums): return base
    if total >= sum(minimums):
        result = [max(n, minimum) for n, minimum in zip(base, minimums)]
        excess = sum(result) - total
        for i in sorted(range(len(result)), key=lambda i: result[i] - minimums[i], reverse=True):
            take = min(excess, result[i] - minimums[i]); result[i] -= take; excess -= take
            if not excess: break
        return result
    # With a small budget, cover distinct route locations before doubling up.
    # Imported detail gain is a preference, never permission to add stations.
    result = [0] * len(components)
    centers = [c['points'][len(c['points'])//2] for c in components]
    weights = [1. + math.sqrt(max(0., c['length'])) + .25*math.log1p(max(0.,float(c['object'].get('gs_detail_gain',0.))))
               for c in components]
    selected = []
    for _ in range(min(total, len(components))):
        def priority(i):
            distance = min(((centers[i]-centers[j]).length for j in selected), default=1.)
            return (weights[i] * (1.+min(3.,distance)), -i)
        index = max((i for i,n in enumerate(result) if n == 0), key=priority)
        result[index] = 1; selected.append(index)
    remaining = total - sum(result)
    if remaining:
        extra = apportion(remaining, [max(0,minimum-result[i]) for i,minimum in enumerate(minimums)])
        result = [n+more for n,more in zip(result,extra)]
    assert sum(result) == total
    return result


def outside_fragments(components, regions, clip):
    fragments = []
    for component in components:
        current = []
        def flush():
            if len(current)>1:
                fragments.append(dict(object=component['object'],points=list(current),
                    length=sum((b-a).length for a,b in zip(current,current[1:]))))
            current.clear()
        for a,b in zip(component['points'],component['points'][1:]):
            delta=b-a;length2=delta.length_squared
            if length2<1e-16:continue
            cuts=[]
            for region in regions:
                part=clip(a,b,region)
                if part:cuts.append(((part[0]-a).dot(delta)/length2,(part[1]-a).dot(delta)/length2))
            cursor=0.;segments=[]
            for lo,hi in sorted(cuts):
                if lo>cursor+1e-8:segments.append((cursor,lo))
                cursor=max(cursor,hi)
            if cursor<1.-1e-8:segments.append((cursor,1.))
            for lo,hi in segments:
                start,end=a.lerp(b,lo),a.lerp(b,hi)
                if current and (current[-1]-start).length>1e-6:flush()
                if not current:current.append(start)
                current.append(end)
                if hi<1.-1e-8:flush()
            if not segments:flush()
        flush()
    return fragments
