#!/usr/bin/env python3
"""Offline mathematics and known evidence limits; no ROS or recorded labels."""
import unittest
import numpy as np
from actual_return_depth import OriginalDepthFrame,ActualReturnDepthFrame


def vector(radius,degrees):
    angle=np.deg2rad(degrees)
    return np.array([radius*np.cos(angle),radius*np.sin(angle),0.])

def fields(q):
    return [q.front,q.behind,q.static,q.unknown,q.nearest]


class ActualReturnQueryTests(unittest.TestCase):
    def assert_query_equal(self,points,query,**kwargs):
        old=OriginalDepthFrame(points,**kwargs).query(query)
        new=ActualReturnDepthFrame(points,**kwargs).query(query)
        for a,b in zip(fields(old),fields(new)):
            np.testing.assert_array_equal(a,b)
    def test_near_query_far_off_ray_returns_unknown(self):
        query=np.array([[.22,.06,0.]])
        returns=np.array([[1.,.4,0.]])
        old=OriginalDepthFrame(returns).query(query)
        new=ActualReturnDepthFrame(returns).query(query)
        self.assertTrue(old.front[0])
        self.assertTrue(new.unknown[0])
        self.assertFalse(new.front[0]);self.assertFalse(new.behind[0]);self.assertFalse(new.static[0])
        self.assertEqual(new.nearest[0],-1)
        angle=np.arccos(float(query[0]@returns[0]/(np.linalg.norm(query[0])*np.linalg.norm(returns[0]))))
        self.assertLess(np.linalg.norm(query[0])*np.sin(angle),.03)
        self.assertGreater(np.linalg.norm(returns[0])*np.sin(angle),.03)
    def test_collinear_foreground_behind_and_real_static_preserved(self):
        for query in ([[.5,0.,0.]],[[1.,0.,0.]],[[1.5,0.,0.]]):
            self.assert_query_equal([[1.,0.,0.]],query)
    def test_actual_return_tube_cutoff(self):
        for lateral,expected in [(.029999,True),(.030001,False)]:
            point=[[np.sqrt(1.-lateral*lateral),lateral,0.]]
            result=ActualReturnDepthFrame(point).query([[.2,0.,0.]])
            self.assertEqual(bool(result.front[0]),expected)
            self.assertEqual(bool(result.unknown[0]),not expected)
    def test_19_and20_degree_neighbors_can_still_qualify(self):
        query=vector(1.,19.)[None,:];returns=vector(1.2,20.)[None,:]
        old=OriginalDepthFrame(returns).query(query)
        new=ActualReturnDepthFrame(returns).query(query)
        self.assertTrue(new.front[0]);self.assert_query_equal(returns,query)
        self.assertLess(1.2*np.sin(np.deg2rad(1.)),.03)
        self.assertFalse(np.isclose(19.,20.))
    def test_orthogonal_and_reverse_returns_are_unknown(self):
        query=[[.001,0.,0.]]
        for point in ([[0.,.001,0.]],[[-.001,0.,0.]]):
            result=ActualReturnDepthFrame(point).query(query)
            self.assertTrue(result.unknown[0])
            self.assertEqual(result.nearest[0],-1)
            self.assertFalse(result.static[0])
    def test_static20cm_sphere_contour_neighbor_can_still_be_front(self):
        # Fixed sphere at(.3,0,0), radius.1:19deg ray hits near surface;
        # adjacent20deg ray misses it and legitimately reaches a1m wall.
        angle=np.deg2rad(19.);center=np.array([.3,0.,0.]);radius=.1
        near=.3*np.cos(angle)-np.sqrt(radius**2-(.3*np.sin(angle))**2)
        query=vector(near,19.)[None,:];returns=vector(1.,20.)[None,:]
        self.assertAlmostEqual(np.linalg.norm(query[0]-center),radius,places=14)
        self.assertGreater(.3*np.sin(np.deg2rad(20.)),radius)
        self.assertLess(np.sin(np.deg2rad(1.)),.03)
        old=OriginalDepthFrame(returns).query(query)
        new=ActualReturnDepthFrame(returns).query(query)
        self.assertTrue(old.front[0]);self.assertTrue(new.front[0])
        self.assert_query_equal(returns,query)
    def test_near_far_empty_interval_is_not_static(self):
        q=ActualReturnDepthFrame([[1.,0.,0.],[3.,0.,0.]]).query([[2.,0.,0.]])
        self.assertTrue(q.behind[0]);self.assertFalse(q.static[0]);self.assertFalse(q.front[0])
        self.assert_query_equal([[1.,0.,0.],[3.,0.,0.]],[[2.,0.,0.]])
    def test_real_near_depth_support_and_nearest_occluder(self):
        q=ActualReturnDepthFrame([[1.01,0.,0.]]).query([[1.,0.,0.]])
        self.assertTrue(q.static[0])
        q=ActualReturnDepthFrame([[.8,0.,0.],[1.01,0.,0.]]).query([[1.,0.,0.]])
        self.assertFalse(q.static[0]);self.assertFalse(q.front[0])
        self.assert_query_equal([[.8,0.,0.],[1.01,0.,0.]],[[1.,0.,0.]])
    def test_empty_or_invalid_view_remains_unknown(self):
        for points in (np.empty((0,3)),[[np.nan,0.,0.]],[[0.,0.,0.]]):
            q=ActualReturnDepthFrame(points).query([[1.,0.,0.]])
            self.assertTrue(q.unknown[0]);self.assertEqual(q.nearest[0],-1)
    def test_invalid_current_queries_remain_unknown(self):
        self.assert_query_equal([[1.,0.,0.]],[[np.nan,0.,0.],[0.,0.,0.]])
    def test_nearer_return_controls_unchanged(self):
        rng=np.random.RandomState(103)
        points=rng.normal(size=(100,3));points=points/np.linalg.norm(points,axis=1)[:,None]*rng.uniform(.2,.9,size=(100,1))
        queries=rng.normal(size=(40,3));queries/=np.linalg.norm(queries,axis=1)[:,None]
        self.assert_query_equal(points,queries)
    def test_duplicate_ties_and_dynamic_flags_unchanged(self):
        self.assert_query_equal([[1.,0.,0.],[1.,0.,0.]],[[1.,0.,0.]],event=np.array([False,True]))
    def test_actual_world_pose_transforms_preserved(self):
        yaw=np.deg2rad(43.)
        rotation=np.array([[np.cos(yaw),-np.sin(yaw),0.],[np.sin(yaw),np.cos(yaw),0.],[0.,0.,1.]])
        origin=np.array([.2,-.4,.6])
        point=origin+np.array([1.,0.,0.])@rotation.T
        queries=origin+np.array([[.7,0.,0.],[1.,0.,0.],[1.2,0.,0.]])@rotation.T
        self.assert_query_equal(point[None,:],queries,rotation=rotation,origin=origin)

    def test_removing_near_static_return_can_make_far_foreground(self):
        # Fewer supporting returns do not make this classification monotonic.
        # This is one query, not a full static-object publication or trajectory.
        sine=.029/.2
        returns=np.array([[.22*np.sqrt(1-sine*sine),.22*sine,0.],
                          [1.,0.,0.]])
        query=np.array([[.2,0.,0.]])
        old=OriginalDepthFrame(returns).query(query)
        new=ActualReturnDepthFrame(returns).query(query)
        self.assertAlmostEqual(.2*sine,.029)
        self.assertAlmostEqual(.22*sine,.0319)
        self.assertTrue(old.static[0]);self.assertFalse(old.front[0])
        self.assertFalse(old.unknown[0]);self.assertEqual(old.nearest[0],0)
        self.assertFalse(new.static[0]);self.assertTrue(new.front[0])
        self.assertFalse(new.unknown[0]);self.assertEqual(new.nearest[0],1)

if __name__=='__main__':unittest.main(verbosity=2)
