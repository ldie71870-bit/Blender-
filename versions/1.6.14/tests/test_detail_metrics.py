import importlib.util,sys,unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location('detail_metrics',Path(__file__).resolve().parents[1]/'detail_metrics.py')
metrics=importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)

class DetailTests(unittest.TestCase):
    def test_repeating_the_same_view_cannot_satisfy_detail(self):
        self.assertFalse(metrics.adequate((0,0,0),[(1,0,0),(1,0,0),(1.1,0,0)]))
        self.assertTrue(metrics.adequate((0,0,0),[(1,-.3,0),(1,.3,0)]))

    def test_short_lines_must_add_marginal_coverage(self):
        targets=[dict(point=(0,0,0)),dict(point=(0,3,0))]
        candidates=[dict(length=1.,observations={0:[(1,-.3,0),(1,.3,0)]}),
                    dict(length=1.,observations={0:[(1,-.4,0),(1,.4,0)]}),
                    dict(length=.8,observations={1:[(1,2.7,0),(1,3.3,0)]})]
        selected,before,after=metrics.select_lines(targets,{0:[],1:[]},candidates,5,.12,12)
        self.assertEqual(len(selected),2)
        self.assertEqual(before,set())
        self.assertEqual(after,{0,1})
        self.assertTrue(all(c['gain']==1 for c in selected))

    def test_spatial_pocket_is_not_outbid_by_surface_texture(self):
        targets=[dict(point=(0,0,0),weight=1000)]
        views=[(1,-.3,0),(1,.3,0)]
        candidates=[dict(model=0,length=1.,kind='DETAIL',observations={0:views}),
                    dict(model=0,length=3.2,kind='GAP',pocket=0,observations={},space_targets=set(range(37))),
                    dict(model=0,length=1.,kind='GAP',pocket=1,observations={0:views},space_targets=set(range(40,45)))]
        selected,before,after=metrics.select_lines(targets,{},candidates,2,.12,12,balanced=True,reserve_gaps=1)
        self.assertEqual(selected[0]['pocket'],0)
        self.assertEqual(selected[0]['space_gain'],37)
        self.assertEqual(selected[0]['gain'],0)
        self.assertEqual(after,{0})

if __name__=='__main__':unittest.main()
