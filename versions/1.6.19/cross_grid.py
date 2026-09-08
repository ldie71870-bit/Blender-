"""Orthogonal long lanes on the same proven-safe layered graph as contours."""
import math
from collections import defaultdict
from . import contour_planner as planner


def lanes(cells,graph,valid,step,spacing=1.2,budget=12):
    if not cells:return [],{'candidates':0,'accepted':0}
    stride=max(2,round(spacing/step));candidates=[]
    for axis in (0,1):
        fixed=1-axis;groups=defaultdict(list)
        low=min(k[fixed] for k in cells)
        for key,cell in cells.items():
            if (key[fixed]-low-stride//2)%stride==0:groups[key[fixed]].append(key)
        for keys in groups.values():
            remaining=set(keys)
            while remaining:
                first=min(remaining,key=lambda k:(cells[k].point[axis],cells[k].point[2]));run=[first];remaining.remove(first)
                while True:
                    neighbors=[k for k,_ in graph.get(run[-1],()) if k in remaining and k[fixed]==first[fixed] and cells[k].point[axis]>cells[run[-1]].point[axis]]
                    if not neighbors:break
                    nxt=min(neighbors,key=lambda k:math.dist(cells[k].point,cells[run[-1]].point))
                    if not valid(cells[run[-1]].point,cells[nxt].point):break
                    run.append(nxt);remaining.remove(nxt)
                points=[cells[k].point for k in run]
                if planner.length(points)>=.8:
                    # Only collapse sections for which the long chord is safe.
                    points=planner.simplify(points,step*.1,valid)
                    route=planner.Route(points,kind='CROSS_GRID');route.grid_axis=axis
                    candidates.append(route)
    # Alternate directions: a long corridor must not consume all lanes of the
    # perpendicular family. No smoothing can turn these straight lanes into loops.
    ordered=[sorted([r for r in candidates if r.grid_axis==axis],key=lambda r:-planner.length(r.points)) for axis in (0,1)]
    selected=[]
    for i in range(max(map(len,ordered),default=0)):
        for group in ordered:
            if i<len(group) and len(selected)<budget:selected.append(group[i])
    return selected,{'candidates':len(candidates),'accepted':len(selected),'axes':[r.grid_axis for r in selected],
                    'length_m':sum(planner.length(r.points) for r in selected)}
