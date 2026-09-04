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

if __name__=='__main__':unittest.main()
