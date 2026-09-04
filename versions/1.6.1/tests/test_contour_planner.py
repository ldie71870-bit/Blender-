import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
package = types.ModuleType('contour_test_addon')
package.__path__ = [str(ROOT)]
sys.modules[package.__name__] = package
from contour_test_addon import contour_planner as planner
from contour_test_addon import path_planner_3d as topology


def fixture(z=1.0, offset=0, hole=True):
    step = .25
    cells = {}
    def point_ok(p):
        x,y,height = p
        return (0 <= x <= 8 and 0 <= y <= 5 and abs(height-z)<1e-6
                and not (hole and 3 < x < 5 and 1.5 < y < 3.5))
    def valid(a,b):
        n = max(1,math.ceil(math.dist(a,b)/.03))
        return all(point_ok(tuple(a[j]+(b[j]-a[j])*i/n for j in range(3))) for i in range(n+1))
    for x in range(33):
        for y in range(21):
            p=(x*step,y*step,z)
            if point_ok(p):
                key=(x,y,offset)
                cells[key]=topology.WalkableCell(key,p,z-1,.2)
    cells,graph=topology.build_walkable_graph(cells.values(),topology.PlannerConfig(step),edge_validator=valid)
    return cells,graph,valid


class ContourTests(unittest.TestCase):
    def test_small_furniture_top_is_not_a_floor(self):
        supports={(i,j,0):(i*.25,j*.25,0) for i in range(25) for j in range(25)}
        supports.update({(i,j,1):(i*.25,j*.25,.75) for i in range(8,13) for j in range(8,13)})
        kept=planner.structural_supports(supports,.25,4.,.38)
        self.assertEqual(len(kept),625)
        self.assertTrue(all(p[2]==0 for p in kept.values()))

    def test_concave_room_with_obstacle_stays_safe_and_connected(self):
        cells,graph,valid=fixture()
        routes,stats=planner.plan_layer(cells,graph,(0,0),.25,.9,valid)
        self.assertGreater(stats['primitive_count'],2)
        self.assertEqual(len(routes),1)
        self.assertGreater(stats['path_coverage_ratio'],.85)
        for route in routes:
            self.assertTrue(all(valid(a,b) for a,b in zip(route.points,route.points[1:])))

    def test_stacked_floors_never_join_by_xy(self):
        cells1,graph1,valid1=fixture()
        cells2,graph2,valid2=fixture(z=4,offset=1)
        cells=cells1|cells2
        graph=graph1|graph2
        valid=lambda a,b: valid1(a,b) or valid2(a,b)
        routes,stats=planner.plan_layer(cells,graph,(0,0),.25,.9,valid)
        self.assertEqual(len(routes),2)
        for route in routes:
            self.assertLess(max(p[2] for p in route.points)-min(p[2] for p in route.points),1e-5)

    def test_diagonal_masks_do_not_become_one_loop(self):
        field=planner.distance_field({(0,0),(1,1)},1)
        loops=planner.isolines(field,.5)
        self.assertEqual(len(loops),2)
        self.assertTrue(all(p[0]==p[-1] for p in loops))

    def test_narrow_corridor_has_a_route(self):
        cells={ (i,0,0):topology.WalkableCell((i,0,0),(i*.25,0,1),0,.2) for i in range(30)}
        valid=lambda a,b: abs(a[1])+abs(b[1])<1e-7 and abs(a[2]-1)+abs(b[2]-1)<1e-7
        cells,graph=topology.build_walkable_graph(cells.values(),topology.PlannerConfig(.25),edge_validator=valid)
        routes,stats=planner.plan_layer(cells,graph,(0,0),.25,.9,valid)
        self.assertEqual(len(routes),1)
        self.assertGreater(planner.length(routes[0].points),6)
        self.assertGreater(stats['path_coverage_ratio'],.99)

    def test_graph_disconnection_is_respected_even_with_permissive_validator(self):
        cells,graph,valid=fixture(hole=False)
        graph={k:[(n,cost) for n,cost in edges if (k[0]<16)==(n[0]<16)] for k,edges in graph.items()}
        routes,stats=planner.plan_layer(cells,graph,(0,0),.25,.9,valid)
        self.assertEqual(len(routes),2)


if __name__=='__main__':
    unittest.main()
