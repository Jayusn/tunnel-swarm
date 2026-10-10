"""Actual-member and sparse-target regressions for the isolated body experiment."""
import unittest

import numpy as np

from event_core import DepthFrame
from density_peak_candidates import dbscan_groups, peak_candidates


def generate(points, types=None, static=None, history=(), eligible=None):
    types = np.zeros(len(points), dtype=np.uint8) if types is None else types
    static = np.zeros(len(points), dtype=bool) if static is None else static
    return peak_candidates(points, types, static, history, eligible=eligible)


class MeasuredBodies(unittest.TestCase):
    def test_twenty_cm_six_returns_and_partial_surface(self):
        angles = np.arange(6) * np.pi / 3
        radius = .1 * np.sin(np.deg2rad(50))
        sphere = np.column_stack([np.full(6, .1 * np.cos(np.deg2rad(50))),
                                  radius * np.cos(angles), radius * np.sin(angles)])
        plane = np.column_stack([.1 * np.cos(angles), .1 * np.sin(angles), np.zeros(6)])
        for points in (sphere, plane, sphere * .1):
            with self.subTest(size=np.ptp(points, axis=0)):
                bodies = generate(points)
                self.assertEqual(len(bodies), 1)
                np.testing.assert_array_equal(bodies[0]['indices'], np.arange(6))
                np.testing.assert_array_equal(bodies[0]['size'], np.ptp(points, axis=0))

    def test_static_and_unknown_points_remain_geometry(self):
        points = np.column_stack([np.zeros(6), np.arange(6) * .002, np.zeros(6)])
        unknown = generate(points)
        stationary = generate(points, static=np.ones(6, dtype=bool))
        np.testing.assert_array_equal(unknown[0]['indices'], stationary[0]['indices'])
        self.assertEqual(stationary[0]['event_fraction'], 0.)
        self.assertEqual(stationary[0]['novelty'], 0.)

    def test_large_parent_has_disjoint_actual_bodies(self):
        rng = np.random.RandomState(27)
        left = rng.uniform(-.02, .02, (410, 3))
        right = rng.uniform(-.02, .02, (410, 3)) + [.4, 0., 0.]
        bridge = np.column_stack([np.arange(.05, .36, .02),
                                  np.zeros(16), np.zeros(16)])
        points = np.vstack([left, bridge, right])
        self.assertEqual(len(dbscan_groups(points)), 1)
        self.assertGreater(len(points), 800)
        bodies = generate(points)
        self.assertGreaterEqual(len(bodies), 2)
        members = np.concatenate([value['indices'] for value in bodies])
        self.assertEqual(len(members), len(np.unique(members)))
        for body in bodies:
            self.assertGreaterEqual(len(body['indices']), 6)
            self.assertLessEqual(len(body['indices']), 800)
            actual = points[body['indices']]
            np.testing.assert_array_equal(body['center'], (actual.min(0) + actual.max(0)) / 2)

    def test_novelty_requires_both_real_histories(self):
        points = np.column_stack([np.zeros(6), np.arange(6) * .002, np.zeros(6)])
        nearby = DepthFrame(points + [.06, 0., 0.])
        farther = DepthFrame(points + [.12, 0., 0.])
        self.assertEqual(generate(points, history=[nearby, farther])[0]['novelty'], 0.)
        self.assertEqual(generate(points, history=[farther, farther])[0]['novelty'], 1.)

    def test_roi_maps_to_original_members_and_batches_evidence(self):
        first = np.column_stack([np.zeros(6), np.arange(6) * .002, np.zeros(6)])
        points = np.vstack([first, first + [1., 0., 0.]])
        eligible = np.arange(12) >= 6
        types = np.array([0] * 6 + [1, 2, 4, 3, 0, 0], dtype=np.uint8)
        bodies = generate(points, types, eligible=eligible)
        self.assertEqual(len(bodies), 1)
        np.testing.assert_array_equal(bodies[0]['indices'], np.arange(6, 12))
        self.assertEqual(bodies[0]['event_counts'], [2, 2, 1])
        self.assertEqual(bodies[0]['event_fraction'], 4 / 6)


if __name__ == '__main__':
    unittest.main()
