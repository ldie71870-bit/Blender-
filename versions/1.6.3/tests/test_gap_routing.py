import sys,types,unittest
from pathlib import Path
from types import SimpleNamespace
package=types.ModuleType('gap_test');package.__path__=[str(Path(__file__).resolve().parents[1])];sys.modules[package.__name__]=package
from gap_test import gap_routing

class GapRoutingTests(unittest.TestCase):
    def test_all_three_branches_of_a_missing_pocket_get_candidates(self):
        keys={(i,0,0) for i in range(-10,11)}|{(0,j,0) for j in range(1,11)}
        cells={k:SimpleNamespace(point=(k[0]*.15,k[1]*.15,.3)) for k in keys}
        graph={k:[(n,.15) for n in keys if abs(k[0]-n[0])+abs(k[1]-n[1])==1] for k in keys}
        routes=gap_routing.pocket_routes(cells,graph,keys,(0,0),.15,lambda a,b:True)
        covered=set().union(*(r['space_keys'] for r in routes))
        self.assertTrue({(-10,0,0),(10,0,0),(0,10,0)}<=covered)
        self.assertTrue(any(r['pass_index']>0 for r in routes))
        self.assertEqual({r['pocket'] for r in routes},{0})
        self.assertTrue(all(.4<=r['length']<=4.2 for r in routes))

    def test_separate_floors_cannot_erase_each_others_gap(self):
        keys={(i,0,z) for i in range(10) for z in (0,300)}
        cells={k:SimpleNamespace(point=(k[0]*.15,0,.3+k[2]/100)) for k in keys}
        graph={k:[(n,.15) for n in keys if k[2]==n[2] and abs(k[0]-n[0])==1] for k in keys}
        routes=gap_routing.pocket_routes(cells,graph,keys,(0,0),.15,lambda a,b:abs(a[2]-b[2])<.1)
        self.assertEqual(len({r['pocket'] for r in routes}),2)
        self.assertTrue(all(len({k[2] for k in r['space_keys']})==1 for r in routes))

if __name__=='__main__':unittest.main()
