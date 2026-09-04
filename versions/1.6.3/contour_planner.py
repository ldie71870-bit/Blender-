"""Contour-first coverage on a layered support graph. No Blender dependency.

Free masks are supplied separately at each camera height. All smoothing and
joining is validated by the adapter; graph distance, never XY proximity alone,
decides whether paths may join. Reuses the previously disconnected 3D planner.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from . import path_planner_3d as topology


@dataclass
class Route:
    points: list
    kind: str = 'CONTOUR'
    closed: bool = False


def structural_supports(supports, step, minimum_area, maximum_rise):
    """Keep broad floor patches and minor patches joining distinct floor heights.

    Furniture branches touching only one floor are not stairs. An explicit floor
    collection in the adapter bypasses this heuristic for small platforms.
    """
    by_xy = {}
    for key in supports:
        by_xy.setdefault(key[:2], []).append(key)
    flat, full = {key:[] for key in supports}, {key:[] for key in supports}
    for key,p in supports.items():
        for dx,dy in ((1,0),(0,1),(1,1),(1,-1)):
            for other in by_xy.get((key[0]+dx,key[1]+dy),()):
                q=supports[other]
                rise=abs(p[2]-q[2])
                if rise <= maximum_rise:
                    cost=math.dist(p,q)
                    full[key].append((other,cost));full[other].append((key,cost))
                    if rise <= .075:
                        flat[key].append((other,cost));flat[other].append((key,cost))
    majors=[c for c in topology._components(flat) if len(c)*step*step >= minimum_area]
    membership={key:i for i,c in enumerate(majors) for key in c}
    means={i:sum(supports[k][2] for k in c)/len(c) for i,c in enumerate(majors)}
    keep=set(membership)
    for component in topology._components(full,set(supports)-keep):
        adjacent={membership[n] for k in component for n,_ in full[k] if n in membership}
        if len(adjacent)>=2 and max(means[n] for n in adjacent)-min(means[n] for n in adjacent)>.45:
            keep.update(component)
    return {key:supports[key] for key in keep}


def length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def distance_field(mask, step):
    """8-neighbour distance to invalid cells, with an explicit outside boundary."""
    mask = set(mask)
    distances, queue = {}, []
    directions = [(x, y, math.hypot(x, y)*step)
                  for x in (-1, 0, 1) for y in (-1, 0, 1) if x or y]
    for key in mask:
        initial = min((cost for dx, dy, cost in directions
                       if (key[0]+dx, key[1]+dy) not in mask), default=math.inf)
        distances[key] = initial
        if math.isfinite(initial):
            heapq.heappush(queue, (initial, key))
    while queue:
        value, key = heapq.heappop(queue)
        if value != distances[key]:
            continue
        for dx, dy, cost in directions:
            other = (key[0]+dx, key[1]+dy)
            if other in mask and value+cost < distances[other]-1e-10:
                distances[other] = value+cost
                heapq.heappush(queue, (value+cost, other))
    return distances


def isolines(field, level):
    """Marching squares in grid coordinates. Diagonal islands stay separate."""
    squares = {(x+dx, y+dy) for x, y in field
               for dx in (-1, 0) for dy in (-1, 0)}
    links, locations = {}, {}
    cases = {1: [(3, 0)], 2: [(0, 1)], 3: [(3, 1)], 4: [(1, 2)],
             5: [(3, 0), (1, 2)], 6: [(0, 2)], 7: [(3, 2)],
             8: [(2, 3)], 9: [(2, 0)], 10: [(0, 1), (2, 3)],
             11: [(2, 1)], 12: [(1, 3)], 13: [(1, 0)], 14: [(0, 3)]}
    for x, y in sorted(squares):
        corners = [(x, y), (x+1, y), (x+1, y+1), (x, y+1)]
        values = [field.get(p, 0.0) for p in corners]
        case = sum(1 << i for i, value in enumerate(values) if value >= level)
        if case not in cases:
            continue
        def crossing(edge):
            a, b = edge, (edge+1) % 4
            t = (level-values[a]) / (values[b]-values[a])
            point = tuple(corners[a][j]+t*(corners[b][j]-corners[a][j]) for j in (0, 1))
            key = tuple(round(v, 7) for v in point)
            locations[key] = point
            return key
        for left, right in cases[case]:
            a, b = crossing(left), crossing(right)
            if a != b:
                links.setdefault(a, set()).add(b)
                links.setdefault(b, set()).add(a)
    unused = {tuple(sorted((a, b))) for a in links for b in links[a]}
    result = []
    while unused:
        edge = min(unused)
        start, current = edge
        unused.remove(edge)
        chain = [start, current]
        while current != start:
            candidates = sorted(n for n in links[current]
                                if tuple(sorted((current, n))) in unused)
            if not candidates:
                break
            other = candidates[0]
            unused.remove(tuple(sorted((current, other))))
            chain.append(other)
            current = other
        if len(chain) >= 3:
            result.append([locations[k] for k in chain])
    return result


def simplify(points, tolerance, valid):
    """Conservative RDP: a simplification must stay collision-free."""
    if len(points) <= 2:
        return list(points)
    def point_distance(p, a, b):
        v = tuple(y-x for x, y in zip(a, b))
        denom = sum(x*x for x in v)
        t = max(0.0, min(1.0, sum((p[i]-a[i])*v[i] for i in range(3))/denom)) if denom else 0
        return math.dist(p, tuple(a[i]+t*v[i] for i in range(3)))
    keep, work = {0, len(points)-1}, [(0, len(points)-1)]
    while work:
        lo, hi = work.pop()
        if hi <= lo+1:
            continue
        distance, index = max((point_distance(points[i], points[lo], points[hi]), i)
                              for i in range(lo+1, hi))
        if distance > tolerance or not valid(points[lo], points[hi]):
            keep.add(index)
            work.extend(((lo, index), (index, hi)))
    return [points[i] for i in sorted(keep)]


def smooth(points, valid, iterations=2):
    """Locally round safe corners; one blocked corner cannot cancel all others."""
    points = list(points)
    for _ in range(iterations):
        if len(points) < 3:
            break
        proposed = [points[0]]
        for i in range(1, len(points)-1):
            a, b, c = points[i-1:i+2]
            left = tuple(.20*a[j]+.80*b[j] for j in range(3))
            right = tuple(.80*b[j]+.20*c[j] for j in range(3))
            if valid(proposed[-1], left) and valid(left, right) and valid(right, c):
                proposed.extend((left, right))
            else:
                proposed.append(b)
        proposed.append(points[-1])
        # Neighbouring corner edits must also be safe together.
        if all(valid(a, b) for a, b in zip(proposed, proposed[1:])):
            points = proposed
    return points


def split_valid(points, valid):
    run = []
    result = []
    for point in points:
        if not valid(point, point):
            if len(run) >= 2:
                result.append(run)
            run = []
            continue
        if run and not valid(run[-1], point):
            if len(run) >= 2:
                result.append(run)
            run = []
        run.append(point)
    if len(run) >= 2:
        result.append(run)
    return result


def nearest_key(point, cells, by_xy, origin, step):
    x = round((point[0]-origin[0])/step)
    y = round((point[1]-origin[1])/step)
    candidates = [k for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                  for k in by_xy.get((x+dx, y+dy), ())]
    return min(candidates, key=lambda k: math.dist(cells[k].point, point), default=None)


def route_anchors(route, cells, by_xy, origin, step, valid):
    indices = range(len(route.points)-1) if route.closed else [0, len(route.points)-1]
    anchors = {}
    for index in indices:
        point = route.points[index]
        key = nearest_key(point, cells, by_xy, origin, step)
        if key is not None and valid(point, cells[key].point):
            anchors.setdefault(key, index)
    return anchors


def join_routes(routes, cells, graph, origin, step, valid, max_bridge):
    """Splice loop excursions via shortest legal graph bridges.

    Dead ends can require retracing. Disconnected graph components can never be
    stitched. There is no unrestricted nearest-endpoint shortcut.
    """
    by_xy = {}
    for key in cells:
        by_xy.setdefault(key[:2], []).append(key)
    work = sorted(routes, key=lambda route: length(route.points), reverse=True)
    # Dense anchors make a pair of separated entry/exit joins possible without
    # forcing both directions of travel through the very same point.
    for route in work:
        if route.closed:
            dense=[]
            for a,b in zip(route.points,route.points[1:]):
                count=max(1,math.ceil(math.dist(a,b)/max(.15,step)))
                dense.extend(tuple(a[j]+(b[j]-a[j])*i/count for j in range(3)) for i in range(count))
            route.points=dense+[route.points[-1]]
    result, bridge_length = [], 0.0
    while work:
        main = work.pop(0)
        while work:
            # Open chains connect only at their ends: never insert an out-and-
            # back excursion into an already visited interior point.
            root = route_anchors(main, cells, by_xy, origin, step, valid)
            targets = {}
            for n, other in enumerate(work):
                for key, index in route_anchors(other, cells, by_xy, origin, step, valid).items():
                    targets.setdefault(key, []).append((n, index))
            queue, dist, previous, source = [], {}, {}, {}
            for key, index in root.items():
                cost = math.dist(main.points[index], cells[key].point)
                dist[key], source[key] = cost, key
                heapq.heappush(queue, (cost, key))
            found = None
            while queue:
                cost, key = heapq.heappop(queue)
                if cost != dist[key] or cost > max_bridge:
                    continue
                if key in targets:
                    n, index = min(targets[key])
                    if cost+math.dist(cells[key].point, work[n].points[index]) <= max_bridge:
                        found = (key, n, index)
                        break
                for neighbor, edge_cost in graph.get(key, ()):
                    candidate = cost+edge_cost
                    if candidate < dist.get(neighbor, math.inf)-1e-9:
                        dist[neighbor], previous[neighbor], source[neighbor] = candidate, key, source[key]
                        heapq.heappush(queue, (candidate, neighbor))
            if found is None:
                break
            key, n, index = found
            other = work.pop(n)
            start = source[key]
            main_index = root[start]
            keys = topology._path(previous, start, key)
            bridge = [main.points[main_index]]+[cells[k].point for k in keys]+[other.points[index]]
            bridge = simplify(bridge, step*.18, valid)
            if main.closed and other.closed and length(bridge)<=3.0:
                joined=splice_loop_pair(main.points,other.points,main_index,index,bridge,valid,step)
                if joined is not None:
                    main.points,travel=joined
                    main.kind='CONTOUR_NETWORK'
                    bridge_length+=travel
                    continue
            if main.closed:
                body = main.points[:-1]
                # Break one short seam and finish at the bridge anchor.
                main.points = body[main_index+1:]+body[:main_index+1]
            elif main_index == 0:
                main.points.reverse()
            if other.closed:
                body = other.points[:-1]
                forward = body[index:]+body[:index]
                backward = [forward[0]]+list(reversed(forward[1:]))
                incoming = main.points[-2] if len(bridge)<2 else bridge[-2]
                def turn_cost(points):
                    a = tuple(points[0][j]-incoming[j] for j in range(3))
                    b = tuple(points[1][j]-points[0][j] for j in range(3))
                    return -sum(a[j]*b[j] for j in range(3))/(math.dist(incoming,points[0])*math.dist(points[0],points[1])+1e-9)
                excursion = min((forward,backward),key=turn_cost)
            else:
                excursion = other.points if index == 0 else list(reversed(other.points))
            main.points = main.points+bridge[1:]+excursion[1:]
            main.closed = False
            main.kind = 'CONTOUR_NETWORK'
            bridge_length += length(bridge)
        result.append(main)
    return result, bridge_length


def splice_loop_pair(main,other,entry,other_entry,bridge,valid,step):
    """Open small seams in two loops and join with separate entry/exit bridges."""
    a=main[:-1];b=other[:-1]
    if min(len(a),len(b))<12:return None
    a=a[entry:]+a[:entry]
    b=b[other_entry:]+b[:other_entry]
    best=None
    def turn(p,q,r):
        left=tuple(q[j]-p[j] for j in range(3));right=tuple(r[j]-q[j] for j in range(3))
        denom=math.sqrt(sum(v*v for v in left)*sum(v*v for v in right))
        if denom<1e-8:return 0.
        return math.acos(max(-1.,min(1.,sum(left[j]*right[j] for j in range(3))/denom)))
    first_bridge=[p for i,p in enumerate(bridge) if not i or math.dist(p,bridge[i-1])>1e-7]
    if len(first_bridge)<2:return None
    for seam in (.45,.65,.85):
        def seam_index(points):
            distance=0.
            for i in range(1,len(points)):
                distance+=math.dist(points[i-1],points[i])
                if distance>=seam:return i
            return len(points)
        ai=seam_index(a)
        if ai>=len(a)//3:continue
        for sign in (1,-1):
            body=b if sign==1 else [b[0]]+list(reversed(b[1:]))
            reverse=[body[0]]+list(reversed(body[1:]))
            cut=seam_index(reverse)
            if cut>=len(body)//3:continue
            long_arc=body[:len(body)-cut+1]
            exit_bridge=[long_arc[-1],a[ai]]
            if math.dist(*exit_bridge)>max(1.,length(first_bridge)*1.8) or not valid(*exit_bridge):continue
            angles=[turn(a[-1],a[0],first_bridge[1]),
                    turn(first_bridge[-2],body[0],body[1]),
                    turn(long_arc[-2],long_arc[-1],a[ai]),
                    turn(long_arc[-1],a[ai],a[(ai+1)%len(a)])]
            if max(angles)>math.radians(135):continue
            points=first_bridge+long_arc[1:]+a[ai:]+[a[0]]
            score=sum(angles)+length(exit_bridge)
            if best is None or score<best[0]:best=(score,points,length(first_bridge)+length(exit_bridge))
    return (best[1],best[2]) if best else None


def covered_keys(routes, cells, graph, radius, origin, step):
    """Coverage on this layer's graph, so walls and overlapping floors cannot leak."""
    by_xy = {}
    for key in cells:
        by_xy.setdefault(key[:2], []).append(key)
    distances, queue = {}, []
    for route in routes:
        for a, b in zip(route.points, route.points[1:]):
            count = max(1, math.ceil(math.dist(a, b)/(step*.7)))
            for i in range(count+1):
                p = tuple(a[j]+(b[j]-a[j])*i/count for j in range(3))
                key = nearest_key(p, cells, by_xy, origin, step)
                if key is None:
                    continue
                cost = math.dist(p, cells[key].point)
                if cost < distances.get(key, math.inf):
                    distances[key] = cost
                    heapq.heappush(queue, (cost, key))
    while queue:
        cost, key = heapq.heappop(queue)
        if cost != distances[key] or cost > radius:
            continue
        for neighbor, edge_cost in graph.get(key, ()):
            value = cost+edge_cost
            if value <= radius and value < distances.get(neighbor, math.inf):
                distances[neighbor] = value
                heapq.heappush(queue, (value, neighbor))
    return {key for key, distance in distances.items() if distance <= radius}


def uncross(points, valid, maximum_swaps=32):
    """Safe 2-opt: reverse a visited arc to remove same-height X crossings."""
    points=list(points)
    for _ in range(maximum_swaps):
        buckets={};pairs=set()
        for i,(a,b) in enumerate(zip(points,points[1:])):
            for x in range(math.floor(min(a[0],b[0])/.5),math.floor(max(a[0],b[0])/.5)+1):
                for y in range(math.floor(min(a[1],b[1])/.5),math.floor(max(a[1],b[1])/.5)+1):
                    for j in buckets.get((x,y),()):
                        if i-j>1:pairs.add((j,i))
                    buckets.setdefault((x,y),[]).append(i)
        options=[]
        for i,j in sorted(pairs):
            a,b=points[i:i+2];c,d=points[j:j+2]
            rx,ry=b[0]-a[0],b[1]-a[1];sx,sy=d[0]-c[0],d[1]-c[1]
            denominator=rx*sy-ry*sx
            if abs(denominator)<1e-9:continue
            qx,qy=c[0]-a[0],c[1]-a[1]
            t=(qx*sy-qy*sx)/denominator;u=(qx*ry-qy*rx)/denominator
            if not (.01<t<.99 and .01<u<.99):continue
            if abs(a[2]+t*(b[2]-a[2])-c[2]-u*(d[2]-c[2]))>.08:continue
            gain=math.dist(a,b)+math.dist(c,d)-math.dist(a,c)-math.dist(b,d)
            if gain>.001:options.append((gain,i,j))
        changed=False
        for _,i,j in sorted(options,reverse=True):
            if valid(points[i],points[j]) and valid(points[i+1],points[j+1]):
                points[i+1:j+1]=reversed(points[i+1:j+1]);changed=True;break
        if not changed:break
    return points


def plan_layer(cells, graph, origin, step, lane_gap, valid, *, minimum_length=.8,
               maximum_bridge=15.0, smoothing=2):
    config = topology.PlannerConfig(grid_spacing=step, units_per_meter=1.0,
                                    minimum_floor_cells=4, minimum_floor_area_m2=max(2.0,step*step*4))
    regions, _membership = topology.identify_floor_regions(cells, graph, config)
    routes = []
    invalid_parts = 0
    for region in regions:
        by_xy = {key[:2]: key for key in region.cell_keys}
        field = distance_field(by_xy, step)
        first = max(step*1.05, lane_gap*.5)
        levels = []
        level = first
        while level < max(field.values(), default=0):
            levels.append(level)
            level += lane_gap
        for level in levels:
            for chain in isolines(field, level):
                points = []
                for x, y in chain:
                    near = [(math.hypot(x-ix, y-iy), key) for ix in (math.floor(x), math.ceil(x))
                            for iy in (math.floor(y), math.ceil(y))
                            if (key := by_xy.get((ix, iy))) is not None]
                    if not near:
                        continue
                    weight = sum(1/max(.01, distance) for distance, key in near)
                    z = sum(cells[key].point[2]/max(.01, distance) for distance, key in near)/weight
                    points.append((origin[0]+x*step, origin[1]+y*step, z))
                pieces = split_valid(points, valid)
                invalid_parts += max(0, len(pieces)-1)
                for piece in pieces:
                    if length(piece) < minimum_length:
                        continue
                    piece = simplify(piece, step*.15, valid)
                    piece = smooth(piece, valid, smoothing)
                    routes.append(Route(piece, closed=math.dist(piece[0], piece[-1]) < 1e-5))
    # Cover thin corridors and residual islands that cannot hold an offset loop.
    radius = max(lane_gap*.7, step*1.4)
    covered = covered_keys(routes, cells, graph, radius, origin, step)
    patch_count = 0
    for keys in topology._components(graph):
        # A component with contour loops already has a main route. Geometric
        # leftovers are evaluated by the independent surface-detail stage;
        # forcing every stub into the main route creates immediate reversals.
        if len(keys) < 3 or any(key in covered for key in keys):
            continue
        ordered = topology._diameter_keys(graph, keys)
        points = [cells[key].point for key in ordered]
        if length(points) < minimum_length:
            continue
        points = simplify(points, step*.2, valid)
        points = smooth(points, valid, smoothing)
        routes.append(Route(points, kind='CORRIDOR_PATCH'))
        patch_count += 1
    primitive_count = len(routes)
    routes, bridge_length = join_routes(routes, cells, graph, origin, step, valid, maximum_bridge)
    # Final validation is a required gate, not a best-effort repair.
    for route in routes:
        route.points=simplify(route.points,min(.04,step*.16),valid)
        route.points=uncross(route.points,valid)
        route.points = smooth(route.points, valid, 1 if smoothing else 0)
        route.points = [p for i, p in enumerate(route.points)
                        if not i or math.dist(p, route.points[i-1]) > 1e-7]
        if not all(valid(a, b) for a, b in zip(route.points, route.points[1:])):
            raise ValueError('Final contour route failed collision validation')
    covered = covered_keys(routes, cells, graph, radius, origin, step)
    stats = dict(floor_regions=len(regions), free_cells=len(cells), covered_cells=len(covered),
                 path_coverage_ratio=len(covered)/max(1, len(cells)), coverage_radius=radius,
                 primitive_count=primitive_count, route_count=len(routes), residual_patches=patch_count,
                 split_contours=invalid_parts, bridge_travel=bridge_length,
                 total_length=sum(length(route.points) for route in routes))
    return routes, stats
