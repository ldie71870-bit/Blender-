"""Reachability across camera heights, without crossing intervening geometry."""
import math
from collections import defaultdict, deque


def reachable_models(models, seed_model, seed_keys, segment_clear):
    by_floor=defaultdict(list)
    for i,model in enumerate(models):
        for key in model['cells']:
            by_floor[key].append(i)
    vertical={}
    reached={(seed_model,k) for k in seed_keys}
    queue=deque(reached)
    while queue:
        i,key=queue.popleft()
        for other,_ in models[i]['graph'].get(key,()):
            node=(i,other)
            if node not in reached:reached.add(node);queue.append(node)
        for j in by_floor[key]:
            node=(j,key)
            if node in reached:continue
            pair=(min(i,j),max(i,j),key)
            if pair not in vertical:
                a=models[i]['cells'][key].point;b=models[j]['cells'][key].point
                vertical[pair]=segment_clear(a,b)
            if vertical[pair]:reached.add(node);queue.append(node)
    return [{k for j,k in reached if j==i} for i in range(len(models))]


def balanced_order(candidates, score, model_count):
    """Round-robin ranked views so an easy height cannot exhaust the pool."""
    groups=[sorted((i for i,c in enumerate(candidates) if c['model']==m),key=score,reverse=True)
            for m in range(model_count)]
    return [g[n] for n in range(max(map(len,groups),default=0)) for g in groups if n<len(g)]
