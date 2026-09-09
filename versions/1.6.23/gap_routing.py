"""Trace residual pockets until their side branches also have candidates."""
import math
from . import contour_planner as planner
from . import path_planner_3d as topology


def pocket_routes(cells, graph, missing, origin, step, valid, radius=.20, max_passes=8):
    """Offer coherent, non-redundant runs, with stable pocket identifiers.

    Each pass removes graph-distance coverage and traces the remaining branches.
    This discovers the other side of a furniture island without a hard-coded ROI.
    """
    result=[]
    pockets=sorted(topology._components(graph,missing),key=lambda k:(-len(k),min(k)))[:40]
    for pocket_id,keys in enumerate(pockets):
        remaining=set(keys)
        for iteration in range(max_passes):
            parts=sorted(topology._components(graph,remaining),key=lambda k:(-len(k),min(k)))
            if not parts:break
            progressed=set()
            for part in parts:
                if len(part)<3:
                    progressed.update(part);continue
                ordered=topology._diameter_keys(graph,part)
                points=[cells[k].point for k in ordered]
                if planner.length(points)<.4:
                    progressed.update(part);continue
                pieces=[];piece=[points[0]];travel=0.
                for point in points[1:]:
                    travel+=math.dist(piece[-1],point);piece.append(point)
                    if travel>=4.0:pieces.append(piece);piece=[point];travel=0.
                if len(piece)>1:pieces.append(piece)
                for piece in pieces:
                    piece=planner.simplify(piece,step*.4,valid)
                    piece=planner.smooth(piece,valid,2)
                    if planner.length(piece)<.4:continue
                    covered=planner.covered_keys([planner.Route(piece)],cells,graph,radius,origin,step)&missing
                    new=covered&remaining
                    if len(new)<3:continue
                    result.append(dict(points=piece,length=planner.length(piece),center=piece[len(piece)//2],
                                       kind='GAP',pocket=pocket_id,space_keys=covered,pass_index=iteration))
                    progressed.update(new)
            if not progressed:break
            remaining-=progressed
    return result
