import json
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rrt_core'))
from local_rrt_core import Scene,Tree,plan
from path_geometry import duration,sample


class PlannerTests(unittest.TestCase):
    def test_exact_segment_collision(self):
        scene=Scene(dict(bounds=[-5,5,-5,5],boxes=[(0,0,1,1)],clearance=0.4))
        self.assertFalse(scene.free((-2,0),(2,0)))  # endpoints free; interior collides
        self.assertFalse(scene.free((0,0),(2,0)))
        self.assertFalse(scene.free((-5,2),(2,2)))
        self.assertTrue(scene.free((-2,2),(2,2)))
        self.assertFalse(scene.free((-2,.9),(2,.9)))  # tangent to inflated box

    def test_closed_boundary_keeps_real_obstacle_clearance(self):
        scene=Scene(dict(bounds=[0,5,0,5],boxes=[(2,2,.2,.2)],
                         boundary_clearance=0),clearance=.3)
        self.assertTrue(scene.free((0,0),(5,0)))
        self.assertFalse(scene.free((0,0),(-.001,0)))
        self.assertFalse(scene.free((0,0),(5.001,0)))
        self.assertFalse(scene.free((1.7,2),(1.7,2)))
        self.assertFalse(scene.free((0,0),(3,3)))

    def test_existing_world_path(self):
        root=Path(__file__).resolve().parents[1]
        scene=Scene(json.loads((root/'config/example_scene.json').read_text()))
        points,stats=plan(scene,scene.data['start'],scene.data['goal'],iterations=3000,timeout=8)
        self.assertEqual(points[0],tuple(scene.data['start']))
        self.assertEqual(points[-1],tuple(scene.data['goal']))
        self.assertGreater(len(points),2)
        self.assertTrue(all(scene.free(a,b) for a,b in zip(points,points[1:])))
        self.assertGreater(stats['length'],math.dist(points[0],points[-1]))

    def test_invalid_start_and_budget_failure(self):
        scene=Scene(dict(bounds=[-5,5,-5,5],boxes=[(0,0,1,1)],clearance=.4))
        with self.assertRaises(ValueError): plan(scene,(0,0),(3,3))
        with self.assertRaises(RuntimeError): plan(scene,(-2,2),(2,2),iterations=0)

    def test_quick_plan_keeps_collision_validation(self):
        root=Path(__file__).resolve().parents[1]
        scene=Scene(json.loads((root/'config/example_scene.json').read_text()))
        points,stats=plan(scene,scene.data['start'],scene.data['goal'],
                          iterations=12000,timeout=8,optimization_time=0)
        self.assertIsNotNone(stats['first_path_time'])
        self.assertTrue(all(scene.free(a,b) for a,b in zip(points,points[1:])))
        self.assertEqual(points[0],tuple(scene.data['start']))
        self.assertEqual(points[-1],tuple(scene.data['goal']))

    def test_rewire_updates_descendants(self):
        t=Tree((0,0))
        t.points.extend([(0,2),(2,2),(2,0)])
        t.parent.extend([0,1,0]);t.cost.extend([2,4,2]);t.children=[{1,3},{2},set(),set()]
        t.reparent(1,3)
        self.assertAlmostEqual(t.cost[1],2+math.sqrt(8))
        self.assertAlmostEqual(t.cost[2],4+math.sqrt(8))

    def test_quintic_stays_on_valid_segment(self):
        a,b=(-2,2),(2,2);T=duration(a,b,.25,.3)
        for i in range(101):
            p,v,acc=sample(a,b,T*i/100,T)
            self.assertTrue(-2-1e-9<=p[0]<=2+1e-9)
            self.assertAlmostEqual(p[1],2)
            self.assertLessEqual(math.hypot(*v),.25+1e-9)
            self.assertLessEqual(math.hypot(*acc),.3+1e-6)
        self.assertEqual(sample(a,b,0,T)[1],[0,0])
        self.assertEqual(sample(a,b,T,T)[1],[0,0])


if __name__=='__main__': unittest.main()
