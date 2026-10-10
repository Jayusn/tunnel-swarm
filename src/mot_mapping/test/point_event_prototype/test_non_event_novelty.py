import unittest

import numpy as np

from event_core import DepthFrame
from density_peak_candidates import peak_candidates
from non_event_novelty import non_event_novelty_tree


class NonEventNovelty(unittest.TestCase):
    def setUp(self):
        self.patch = np.array([[1., y, 0.] for y in [-.025, -.015, -.005, .005, .015, .025]])
        self.wall = self.patch + [1., 0., 0.]

    def modified(self, points, event, eligible=None):
        frame = DepthFrame(points, event=event, eligible=eligible)
        frame.world_tree = non_event_novelty_tree(frame.points, frame.event, eligible)
        return frame

    def novelty(self, history):
        values = peak_candidates(self.patch, np.ones(6, dtype=np.uint8),
                                 np.zeros(6, dtype=bool), history)
        self.assertEqual(len(values), 1)
        np.testing.assert_array_equal(values[0]['indices'], np.arange(6))
        return values[0]['novelty']

    def test_actual_moving_history_is_not_stationary_novelty_reference(self):
        points = np.vstack([self.patch, self.wall])
        event = np.array([True] * 6 + [False] * 6)
        raw = DepthFrame(points, event=event)
        filtered = self.modified(points, event)
        self.assertEqual(self.novelty([raw, raw]), 0.)
        self.assertEqual(self.novelty([filtered, filtered]), 1.)

    def test_unknown_non_event_returns_still_block_novelty(self):
        filtered = self.modified(np.vstack([self.patch, self.wall]), np.zeros(12, dtype=bool))
        self.assertEqual(self.novelty([filtered, filtered]), 0.)

    def test_missing_or_empty_reference_cannot_supply_motion(self):
        empty = self.modified(self.patch, np.ones(6, dtype=bool))
        valid = self.modified(self.wall, np.zeros(6, dtype=bool))
        self.assertIsNone(empty.world_tree)
        self.assertEqual(self.novelty([valid, empty]), 0.)
        self.assertEqual(self.novelty([valid]), 0.)

    def test_roi_and_ray_geometry_are_independent(self):
        points = np.vstack([self.patch, self.wall])
        event = np.array([True] * 6 + [False] * 6)
        frame = DepthFrame(points, event=event)
        ray_before = frame.query(self.patch)
        points_before = frame.points.copy()
        frame.world_tree = non_event_novelty_tree(frame.points, event, np.array([True] * 6 + [False] * 6))
        self.assertIsNone(frame.world_tree)
        np.testing.assert_array_equal(frame.points, points_before)
        ray_after = frame.query(self.patch)
        for field in ('front', 'behind', 'static', 'unknown', 'nearest'):
            np.testing.assert_array_equal(getattr(ray_before, field), getattr(ray_after, field))

    def test_two_real_references_remain_required_at_six_cm_boundary(self):
        farther = self.modified(self.wall, np.zeros(6, dtype=bool))
        # As in the existing raw-reference regression, points at/below the
        # existing floating-point .06 boundary cannot count as new.
        at_origin = self.patch - [1., 0., 0.]
        close = self.modified(at_origin + [.06, 0., 0.], np.zeros(6, dtype=bool))
        values = peak_candidates(at_origin, np.ones(6, dtype=np.uint8), np.zeros(6, dtype=bool), [close, farther])
        self.assertEqual(values[0]['novelty'], 0.)


if __name__ == '__main__':
    unittest.main()
