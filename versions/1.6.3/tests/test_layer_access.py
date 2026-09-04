import sys,types,unittest
from pathlib import Path
from types import SimpleNamespace
package=types.ModuleType('layer_access_test');package.__path__=[str(Path(__file__).resolve().parents[1])];sys.modules[package.__name__]=package
from layer_access_test import layer_access,detail_metrics

class LayerAccessTests(unittest.TestCase):
    def test_visible_surface_does_not_erase_a_missing_spatial_pocket(self):
        targets=[dict(point=(0,0,0),weight=1)]
        views=[(-1,1,0),(1,1,0)]
        candidate=dict(model=0,length=2,observations={0:views},space_targets={(0,i,0,0) for i in range(8)})
        selected,before,after=detail_metrics.select_lines(targets,{0:views},[candidate],1,.12,12,balanced=True,reserve_gaps=2)
        self.assertEqual(len(selected),1)
        self.assertEqual(selected[0]['gain'],0)
        self.assertEqual(selected[0]['space_gain'],8)
        self.assertEqual(before,after)

    def test_low_pocket_reached_via_middle_but_not_through_a_ceiling(self):
        keys=[(0,0,0),(1,0,0)]
        models=[dict(cells={k:SimpleNamespace(point=(k[0],0,z)) for k in keys},
                     graph={keys[0]:[(keys[1],1)] if i==1 else [],keys[1]:[(keys[0],1)] if i==1 else []}) for i,z in enumerate((.3,1.2,3.3))]
        reached=layer_access.reachable_models(models,0,[keys[0]],lambda a,b:max(a[2],b[2])<3)
        self.assertEqual(reached[0],set(keys))
        self.assertEqual(reached[1],set(keys))
        self.assertEqual(reached[2],set())

    def test_small_low_gain_cannot_be_starved_by_high_layer(self):
        targets=[dict(point=(i*2,0,0),weight=1) for i in range(5)]
        views=lambda i:[(i*2-1,1,0),(i*2+1,1,0)]
        candidates=[dict(model=0,length=1,observations={0:views(0)}),
                    dict(model=1,length=1,observations={i:views(i) for i in range(1,4)}),
                    dict(model=1,length=1,observations={4:views(4)})]
        selected,_,_=detail_metrics.select_lines(targets,{},candidates,2,.12,12,balanced=True)
        self.assertEqual({c['model'] for c in selected},{0,1})

if __name__=='__main__':unittest.main()
