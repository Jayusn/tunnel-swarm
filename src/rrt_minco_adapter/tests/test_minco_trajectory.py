import math
import json
from unittest.mock import patch
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rrt_core'))
from local_rrt_core import Scene
from minco_trajectory import MincoTrajectory, speed_penalty_value_gradient, compact_halfspaces, bernstein_norm_bound, basis, coefficient_map, certify_curve, corridor, curve_length_value_gradient
from trajectory_backend import build_trajectory, backend_stats


class MincoTests(unittest.TestCase):
    def test_corner_start_with_closed_boundary(self):
        scene=Scene(dict(bounds=[0,5,0,5],boxes=[],boundary_clearance=0),clearance=.3)
        from local_rrt_core import plan
        route,_=plan(scene,(0.,0.),(3.,3.),seed=7,iterations=12000,timeout=8,optimization_time=.5)
        trajectory=build_trajectory(route,scene,.2,.8,
                                    time_optimization=False, pair_search=False,
                                    terminal_braking=False,start_turn_braking=False,
                                    position_length_weight=1.,position_speed_weight=1.,
                                    position_ftol=1e-8,compact_constraints=True,improve_initial=True)
        self.assertTrue(all(certify_curve(c,scene) for c in trajectory.control_points))
        self.assertFalse(certify_curve(np.array([[-.001,0.],[1.,1.],[3.,3.]]),scene))
        from smooth_trajectory import curve_free
        self.assertTrue(curve_free([(0,0),(1,1),(3,3)],scene))
        self.assertFalse(curve_free([(0,0),(-1,1),(3,3)],scene))

    def test_subdivision_bounds_interior_peak_without_sampling(self):
        # 二次曲线 x(u)=2u(1-u)，端点为零，内部真实峰值为 0.5。
        controls=np.array([[0.,0.],[1.,0.],[0.,0.]])
        coarse=bernstein_norm_bound(controls,0)
        tight=bernstein_norm_bound(controls,4)
        self.assertGreaterEqual(tight,.5)
        self.assertLess(tight,.50000001)
        self.assertLess(tight,coarse)

    def test_subdivision_random_derivatives_contain_dense_samples(self):
        rng=np.random.default_rng(17)
        for degree in (3,4):
            for _ in range(8):
                controls=rng.normal(size=(degree+1,2))
                bounds=[bernstein_norm_bound(controls,d) for d in range(5)]
                self.assertTrue(all(b<=a+1e-12 for a,b in zip(bounds,bounds[1:])))
                u=np.linspace(0,1,2001)
                weights=np.array([math.comb(degree,k)*u**k*(1-u)**(degree-k)
                                  for k in range(degree+1)]).T
                self.assertLessEqual(np.linalg.norm(weights@controls,axis=1).max(),bounds[-1]+1e-12)
        self.assertAlmostEqual(bernstein_norm_bound(np.zeros((5,2))),0.,places=12)

    def test_halfspace_reduction_preserves_tightest_parallel_bound(self):
        a=np.array([[1.,0.],[1.,0.],[-1.,0.],[0.,1.],[1.,1e-12]])
        b=np.array([2.,1.,2.,3.,4.])
        c,d=compact_halfspaces(a,b)
        self.assertEqual(len(d),4)
        self.assertEqual(d[0],1.)
        for x in np.random.default_rng(7).uniform(-4,4,size=(1000,2)):
            self.assertEqual(bool(np.all(a@x<=b)),bool(np.all(c@x<=d)))
        self.assertTrue(np.any(c[:,1]==1e-12))

    def test_position_timing_and_improved_initial_metadata(self):
        tr=build_trajectory([(-2,0),(-2,2),(2,2),(2,0)],self.scene([(0,0,1,1)]),time_optimization=False)
        r=tr.time_optimization['baseline_fit']
        self.assertLessEqual(r['constraints_after'],r['constraints_before'])
        self.assertLessEqual(r['initial_objective_after'],r['initial_objective_before'])
        self.assertGreater(r['solver_gradient_calls'],0)
        self.assertGreaterEqual(r['objective_gradient_evaluations'],r['solver_objective_calls'])
        self.assertEqual(r['position_ftol'],1e-8)
        self.assertTrue(tr.constraint_checks['corridor_valid'])

    def test_speed_penalty_analytic_gradient_and_inactive_region(self):
        rng=np.random.default_rng(23)
        knots=rng.normal(size=(4,2));mapping=rng.normal(size=(12,4))
        weights=np.ones(12)/12
        value,grad=speed_penalty_value_gradient(knots,mapping,weights,.6)
        self.assertGreater(value,0.)
        for i in range(4):
            for j in range(2):
                plus=knots.copy();minus=knots.copy();plus[i,j]+=1e-6;minus[i,j]-=1e-6
                diff=(speed_penalty_value_gradient(plus,mapping,weights,.6)[0]-
                      speed_penalty_value_gradient(minus,mapping,weights,.6)[0])/2e-6
                self.assertAlmostEqual(grad[i,j],diff,delta=max(1e-6,abs(diff)*1e-6))
        value,grad=speed_penalty_value_gradient(knots,np.zeros((12,4)),weights,.6)
        self.assertEqual(value,0.);np.testing.assert_array_equal(grad,0.)
        value,grad=speed_penalty_value_gradient(np.array([[.6,0.]]),np.ones((1,1)),np.ones(1),.6)
        self.assertEqual(value,0.);np.testing.assert_array_equal(grad,0.)

    def test_speed_penalty_preserves_continuous_validation_and_reports(self):
        scene=self.scene([(0,0,1,1)])
        tr=build_trajectory([(-2,0),(-2,2),(2,2),(2,0)],scene,
                            time_optimization=False,position_speed_weight=1.,terminal_braking=True)
        r=tr.time_optimization
        self.assertEqual(r['position_speed_weight'],1.)
        self.assertEqual(r['baseline_fit']['speed_penalty_weight'],1.)
        self.assertEqual(r['evaluations'],0)
        self.assertTrue(all(certify_curve(c,scene) for c in tr.control_points))
        for t in np.linspace(0,tr.total_time,1001):
            self.assertLessEqual(np.linalg.norm(tr.derivative(t,1)),.6+1e-9)
            self.assertLessEqual(np.linalg.norm(tr.derivative(t,2)),.8+1e-9)
        for i in range(len(tr.durations)-1):
            for d in range(5):
                np.testing.assert_allclose(basis(1,d)@tr.coefficients[i]/tr.durations[i]**d,
                    basis(0,d)@tr.coefficients[i+1]/tr.durations[i+1]**d,atol=1e-8)
        json.dumps(r,allow_nan=False)
        for weight in (-1.,float('nan')):
            with self.assertRaises(ValueError):
                build_trajectory([(-2,0),(2,0)],self.scene(),position_speed_weight=weight)

    def scene(self, boxes=()):
        return Scene(dict(bounds=[-5,5,-5,5],boxes=boxes,clearance=.4))

    def test_single_piece_matches_analytic_minimum_jerk(self):
        coefficients=coefficient_map([2.])@np.array([[0.,0.],[1.,2.]])
        expected=np.array([0.,0.,0.,10.,-15.,6.])
        np.testing.assert_allclose(coefficients[:,0],expected,atol=1e-11)
        np.testing.assert_allclose(coefficients[:,1],2*expected,atol=1e-11)

    def test_nonuniform_map_has_minimum_control_continuity(self):
        times=np.array([.7,1.3,.9])
        knots=np.array([[0.,0.],[1.,.2],[1.4,1.],[2.,1.5]])
        coeff=(coefficient_map(times)@knots).reshape(-1,6,2)
        for i in range(len(times)-1):
            for derivative in range(5):
                np.testing.assert_allclose(
                    basis(1,derivative)@coeff[i]/times[i]**derivative,
                    basis(0,derivative)@coeff[i+1]/times[i+1]**derivative,atol=1e-9)
        for c,T,u in ((coeff[0],times[0],0.),(coeff[-1],times[-1],1.)):
            for derivative in (1,2):
                np.testing.assert_allclose(basis(u,derivative)@c/T**derivative,0,atol=1e-10)

    def test_corridor_route_continuity_and_certified_dynamics(self):
        scene=self.scene([(0,0,1,1)])
        tr=MincoTrajectory([(-2,0),(-2,2),(2,2),(2,0)],scene)
        self.assertTrue(all(certify_curve(c,scene) for c in tr.control_points))
        self.assertLessEqual(tr.velocity_bound,.6)
        self.assertLessEqual(tr.acceleration_bound,.8)
        np.testing.assert_allclose(tr.sample(0)[0],(-2,0),atol=1e-9)
        np.testing.assert_allclose(tr.sample(tr.total_time)[0],(2,0),atol=1e-9)
        for t in (0.,tr.total_time):
            for d in (1,2): np.testing.assert_allclose(tr.derivative(t,d),0,atol=1e-9)
        for i in range(len(tr.durations)-1):
            for d in range(5):
                np.testing.assert_allclose(
                    basis(1,d)@tr.coefficients[i]/tr.durations[i]**d,
                    basis(0,d)@tr.coefficients[i+1]/tr.durations[i+1]**d,atol=1e-8)
        for t in np.linspace(0,tr.total_time,1001):
            p,v=tr.sample(t)
            self.assertTrue(scene.free(p,p))
            self.assertLessEqual(math.hypot(*v),.6+1e-9)
            self.assertLessEqual(np.linalg.norm(tr.derivative(t,2)),.8+1e-9)

    def test_segment_time_improves_and_metadata_is_serializable(self):
        scene=self.scene([(0,0,1,1)])
        path=[(-2,0),(-2,2),(2,2),(2,0)]
        baseline=MincoTrajectory(path,scene,time_optimization=False)
        tr=MincoTrajectory(path,scene)
        report=backend_stats(tr)['time_optimization']
        json.dumps(report,allow_nan=False)
        self.assertEqual(len(report['history']),report['evaluations'])
        for name,seconds in report['fit_stage_totals_s'].items():
            self.assertAlmostEqual(seconds,report['baseline_fit']['stage_wall_s'].get(name,0.)+
                sum(h['fit']['stage_wall_s'].get(name,0.) for h in report['history']))
        self.assertAlmostEqual(report['before']['duration_s'],baseline.total_time)
        self.assertLess(tr.total_time,baseline.total_time)
        self.assertGreater(report['accepted'],0)
        self.assertEqual(report['coordinate_evaluation_budget'],8)
        self.assertLessEqual(report['evaluations'],8)
        ratios=tr.durations/baseline.durations
        self.assertGreater(float(np.ptp(ratios)),1e-3)
        for stage in ('before','after'):
            checks=report[stage]['constraints']
            for name in ('continuous_collision_free','corridor_valid','velocity_valid','acceleration_valid'):
                self.assertTrue(checks[name])

    def test_failed_time_candidates_preserve_certified_baseline(self):
        path=[(-2,0),(2,0)];scene=self.scene()
        baseline=MincoTrajectory(path,scene,time_optimization=False)
        original=MincoTrajectory._fit
        calls=[]
        def fail_candidates(obj,*args):
            calls.append(True)
            if len(calls)>1: raise ValueError('injected candidate failure')
            return original(obj,*args)
        with patch.object(MincoTrajectory,'_fit',fail_candidates):
            tr=MincoTrajectory(path,scene,time_evaluations=4,pair_evaluations=0)
        np.testing.assert_allclose(tr.coefficients,baseline.coefficients)
        np.testing.assert_allclose(tr.durations,baseline.durations)
        self.assertEqual(tr.time_optimization['status'],'baseline_retained')
        self.assertEqual(tr.time_optimization['infeasible_candidates'],4)

    def test_warm_start_skips_lp_and_keeps_certified_solution(self):
        scene=self.scene()
        guides=np.array([[-2.,0.],[0.,0.],[2.,0.]])
        corridors=[corridor(a,b,scene) for a,b in zip(guides,guides[1:])]
        cold=object.__new__(MincoTrajectory)
        cold._fit(guides,corridors,np.array([4.,4.]),scene,.6,.8)
        warm=object.__new__(MincoTrajectory)
        with patch('minco_trajectory.linprog',side_effect=AssertionError('unnecessary LP')):
            warm._fit(guides,corridors,np.array([4.,4.]),scene,.6,.8,cold.optimized_knots)
        self.assertTrue(warm.fit_diagnostics['warm_start_used'])
        self.assertFalse(warm.fit_diagnostics['lp_called'])
        np.testing.assert_allclose(warm.coefficients,cold.coefficients,atol=1e-8)
        self.assertTrue(all(certify_curve(c,scene) for c in warm.control_points))

    def test_infeasible_initial_positions_fall_back_to_lp(self):
        scene=self.scene()
        guides=np.array([[-2.,0.],[0.,0.],[2.,0.]])
        corridors=[corridor(a,b,scene) for a,b in zip(guides,guides[1:])]
        bad=guides.copy();bad[1]=[100.,100.]
        tr=object.__new__(MincoTrajectory)
        tr._fit(bad,corridors,np.array([4.,4.]),scene,.6,.8,bad)
        self.assertFalse(tr.fit_diagnostics['warm_start_used'])
        self.assertTrue(tr.fit_diagnostics['lp_called'])
        self.assertEqual(tr.fit_diagnostics['initial_source'],'linear_program')
        self.assertTrue(tr.constraint_checks['continuous_collision_free'])

    def test_timing_includes_failed_candidate_and_serializes(self):
        scene=self.scene();guides=np.array([[-2.,0.],[0.,0.],[2.,0.]])
        corridors=[corridor(a,b,scene) for a,b in zip(guides,guides[1:])]
        tr=object.__new__(MincoTrajectory)
        with patch('minco_trajectory.minimize',side_effect=ValueError('injected QP failure')):
            with self.assertRaises(ValueError):
                tr._fit(guides,corridors,np.array([4.,4.]),scene,.6,.8)
        report=tr.fit_diagnostics
        self.assertFalse(report['success'])
        self.assertIn('position_qp',report['stage_wall_s'])
        self.assertLessEqual(sum(report['stage_wall_s'].values()),report['wall_time_s'])
        self.assertTrue(all(t>=0 for t in report['stage_wall_s'].values()))
        json.dumps(report,allow_nan=False)

    def test_pair_transfer_preserves_time_and_minimum_duration(self):
        raw=np.array([1.,.31,2.])
        changed=MincoTrajectory._paired_durations(raw,0,1,.12)
        self.assertAlmostEqual(changed.sum(),raw.sum())
        self.assertGreater(changed[0],raw[0])
        self.assertLess(changed[1],raw[1])
        self.assertGreaterEqual(changed.min(),.3-1e-12)
        np.testing.assert_array_equal(raw,[1.,.31,2.])

    def test_pair_budget_metadata_and_no_regression_against_pair_entry(self):
        tr=MincoTrajectory([(-2,0),(-2,2),(2,2),(2,0)],self.scene([(0,0,1,1)]),time_evaluations=32,pair_search=True)
        report=tr.time_optimization;pair=report['pair_search']
        entries=[h for h in report['history'] if h['search']=='pair']
        self.assertEqual(pair['evaluation_budget'],8)
        self.assertEqual(pair['evaluations'],len(entries))
        self.assertEqual(report['evaluations'],40)
        self.assertLessEqual(pair['after']['duration_s'],pair['before']['duration_s']+1e-9)
        for entry in entries:
            self.assertNotEqual(entry['bottleneck_segment'],entry['donor_segment'])
            self.assertGreater(entry['transferred_s'],0)
        old=MincoTrajectory([(-2,0),(2,0)],self.scene(),pair_search=False)
        self.assertFalse(old.time_optimization['pair_search']['enabled'])
        self.assertTrue(all(h['search']=='coordinate' for h in old.time_optimization['history']))

    def test_failed_pairs_preserve_complete_coordinate_result(self):
        path=[(-2,0),(2,0)];scene=self.scene()
        old=MincoTrajectory(path,scene,pair_search=False,time_evaluations=4)
        original=MincoTrajectory._fit;calls=[]
        def fail_pairs(obj,*args):
            calls.append(True)
            if len(calls)>5: raise ValueError('injected pair failure')
            return original(obj,*args)
        with patch.object(MincoTrajectory,'_fit',fail_pairs):
            tr=MincoTrajectory(path,scene,time_evaluations=4,pair_evaluations=2,pair_search=True)
        np.testing.assert_allclose(tr.coefficients,old.coefficients)
        np.testing.assert_allclose(tr.durations,old.durations)
        self.assertEqual(tr.time_optimization['pair_search']['accepted'],0)
        self.assertEqual(tr.time_optimization['evaluations'],6)

    def test_curve_length_gradient_matches_finite_difference(self):
        durations=np.array([1.,1.4,1.1]);knots=np.array([[-2.,0.],[-1.,1.],[1.,1.],[2.,0.]])
        nodes,weights=np.polynomial.legendre.leggauss(24)
        mapping=coefficient_map(durations).reshape(3,6,4)
        derivative_map=np.einsum('qk,pkn->pqn',np.array([basis((u+1)/2,1) for u in nodes]),mapping).reshape(-1,4)
        weights=np.tile(weights/2,3)
        value,grad=curve_length_value_gradient(knots,derivative_map,weights)
        self.assertGreater(value,np.linalg.norm(knots[-1]-knots[0]))
        for i in range(4):
            for j in range(2):
                plus=knots.copy();minus=knots.copy();plus[i,j]+=1e-6;minus[i,j]-=1e-6
                numerical=(curve_length_value_gradient(plus,derivative_map,weights)[0]-
                           curve_length_value_gradient(minus,derivative_map,weights)[0])/2e-6
                self.assertAlmostEqual(grad[i,j],numerical,places=6)

    def test_length_objective_shortens_fixed_time_fit_and_preserves_safety(self):
        scene=self.scene([(0,0,1,1)]);path=[(-2,0),(-2,2),(2,2),(2,0)]
        old=MincoTrajectory(path,scene,time_optimization=False,position_length_weight=0.)
        new=MincoTrajectory(path,scene,time_optimization=False,position_length_weight=1.)
        self.assertLess(new.s[-1],old.s[-1])
        self.assertTrue(all(certify_curve(c,scene) for c in new.control_points))
        self.assertLessEqual(new.velocity_bound,.6)
        self.assertLessEqual(new.acceleration_bound,.8)
        self.assertEqual(new.time_optimization['position_length_weight'],1.)
        json.dumps(new.time_optimization,allow_nan=False)
        for weight in (-1.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):MincoTrajectory(path,scene,position_length_weight=weight)

    def test_terminal_braking_preserves_safety_continuity_and_growth_limits(self):
        tr=MincoTrajectory([(-2,0),(-2,2),(2,2),(2,0)],self.scene([(0,0,1,1)]),terminal_braking=True)
        report=tr.time_optimization['terminal_braking']
        self.assertLessEqual(tr.total_time,report['before']['duration_s']+.8+1e-9)
        self.assertLessEqual(tr.s[-1],report['before']['length_m']*1.005+1e-9)
        self.assertTrue(all(certify_curve(c,self.scene([(0,0,1,1)])) for c in tr.control_points))
        if report['accepted']:
            self.assertLessEqual(report['acceleration_bound_after'],report['acceleration_bound_before']*.9)
        for i in range(len(tr.durations)-1):
            for d in range(5):
                np.testing.assert_allclose(basis(1,d)@tr.coefficients[i]/tr.durations[i]**d,
                    basis(0,d)@tr.coefficients[i+1]/tr.durations[i+1]**d,atol=1e-7)
        for d in (1,2):np.testing.assert_allclose(tr.derivative(tr.total_time,d),0,atol=1e-8)
        json.dumps(report,allow_nan=False)

    def test_terminal_failed_candidates_leave_original_curve(self):
        path=[(-2,0),(2,0)];scene=self.scene()
        old=MincoTrajectory(path,scene,time_evaluations=0)
        original=MincoTrajectory._fit;calls=[]
        def fail_extra(obj,*args):
            calls.append(True)
            if len(calls)>1:raise ValueError('terminal failure')
            return original(obj,*args)
        with patch.object(MincoTrajectory,'_fit',fail_extra):
            tr=MincoTrajectory(path,scene,time_evaluations=0,terminal_braking=True)
        np.testing.assert_array_equal(tr.coefficients,old.coefficients)
        np.testing.assert_array_equal(tr.durations,old.durations)
        self.assertFalse(tr.time_optimization['terminal_braking']['accepted'])

    def test_start_turn_braking_limits_and_continuity(self):
        scene=self.scene();tr=MincoTrajectory([(-2,-2),(-2,0),(1,0),(2,2)],scene,
                                             terminal_braking=True,start_turn_braking=True)
        r=tr.time_optimization['start_turn_braking']
        self.assertIn('corner_xy',r)
        self.assertLessEqual(tr.total_time,r['before']['duration_s']+1.5+1e-9)
        self.assertLessEqual(tr.s[-1],r['before']['length_m']*1.01+1e-9)
        if r['accepted']:
            self.assertLessEqual(r['speed_bound_after'],r['speed_bound_before']*.95)
            self.assertLessEqual(r['acceleration_bound_after'],r['acceleration_bound_before']*.85)
        self.assertTrue(all(certify_curve(c,scene) for c in tr.control_points))
        for i in range(len(tr.durations)-1):
            for d in range(5):
                np.testing.assert_allclose(basis(1,d)@tr.coefficients[i]/tr.durations[i]**d,
                    basis(0,d)@tr.coefficients[i+1]/tr.durations[i+1]**d,atol=1e-7)
        self.assertLessEqual(tr.velocity_bound,.6)
        self.assertLessEqual(tr.acceleration_bound,.8)
        json.dumps(tr.time_optimization,allow_nan=False)

    def test_straight_route_skips_start_turn_braking(self):
        scene=self.scene();old=MincoTrajectory([(-2,0),(2,0)],scene)
        tr=MincoTrajectory([(-2,0),(2,0)],scene,start_turn_braking=True)
        self.assertEqual(tr.time_optimization['start_turn_braking']['reason'],'no_early_sharp_turn')
        np.testing.assert_array_equal(tr.coefficients,old.coefficients)
        np.testing.assert_array_equal(tr.durations,old.durations)

    def test_invalid_inputs_and_colliding_route_rejected(self):
        scene=self.scene([(0,0,1,1)])
        for path in ([(-2,0),(2,0)],[(0,0),(0,0)],[(float('nan'),0),(2,0)]):
            with self.assertRaises(ValueError): MincoTrajectory(path,scene)
        for value in (0,-1,float('inf'),float('nan')):
            with self.assertRaises(ValueError):
                MincoTrajectory([(-2,2),(2,2)],scene,speed=value)
        self.assertFalse(certify_curve(np.array([[-2.,0.],[0.,0.],[2.,0.]]),scene))

    def test_default_backend_and_explicit_legacy(self):
        scene=self.scene();path=[(-2,0),(2,0)]
        tr=build_trajectory(path,scene)
        self.assertIsInstance(tr,MincoTrajectory)
        self.assertIn('MINCO-S3',backend_stats(tr)['backend'])
        self.assertEqual(type(build_trajectory(path,scene,backend='bezier')).__name__,'SmoothTrajectory')
        with self.assertRaises(ValueError): build_trajectory(path,scene,backend='typo')


if __name__=='__main__': unittest.main()
