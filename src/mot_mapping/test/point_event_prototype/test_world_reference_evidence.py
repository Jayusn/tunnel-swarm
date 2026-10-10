import unittest

import numpy as np
from scipy.spatial import cKDTree

from event_core import DepthFrame
from world_reference_evidence import actual_view_reference, reference_candidate_features


class WorldReference(unittest.TestCase):
    def setUp(self):
        self.patch = np.array([[1., y, 0.] for y in [-.025, -.015, -.005, .005, .015, .025]])
        self.body = {'indices': np.arange(6), 'center': self.patch.mean(0),
                     'size': np.ptp(self.patch, axis=0), 'novelty': 0., 'event_fraction': 0.}

    def evidence(self, reference):
        frame = DepthFrame(reference)
        return reference_candidate_features(self.patch, [self.body], frame, frame.world_tree)[0]

    def test_far_real_returns_supply_foreground_without_size_filter(self):
        value = self.evidence(self.patch * 2)
        self.assertEqual(value['reference_front_fraction'], 1.)
        self.assertEqual(value['reference_novelty'], 1.)
        np.testing.assert_array_equal(value['indices'], np.arange(6))
        np.testing.assert_array_equal(value['size'], self.body['size'])
        self.assertEqual(value['event_fraction'], 0.)
        self.assertEqual(value['novelty'], 0.)

    def test_missing_return_cannot_supply_foreground_or_novelty(self):
        value = self.evidence([])
        self.assertEqual(value['reference_front_fraction'], 0.)
        self.assertEqual(value['reference_novelty'], 0.)

    def test_nearer_occluder_blocks_farther_layer_foreground(self):
        for reference in (self.patch * .5, np.vstack([self.patch * .5, self.patch * 2])):
            value = self.evidence(reference)
            self.assertEqual(value['reference_front_fraction'], 0.)
            self.assertEqual(value['reference_novelty'], 1.)

    def test_actual_static_return_blocks_foreground(self):
        value = self.evidence(np.vstack([self.patch, self.patch * 2]))
        self.assertEqual(value['reference_front_fraction'], 0.)
        self.assertEqual(value['reference_novelty'], 0.)

    def test_unknown_view_stays_unknown_after_sensor_moves(self):
        reference = self.patch * 2
        view = DepthFrame(reference, origin=[0., .5, 0.])
        value = reference_candidate_features(self.patch, [self.body], view, cKDTree(reference))[0]
        self.assertEqual(value['reference_front_fraction'], 0.)
        self.assertEqual(value['reference_novelty'], 1.)

    def test_reference_reduction_keeps_actual_returns_and_input_unchanged(self):
        reference = np.vstack([self.patch, self.patch + [.0001, 0., 0.]])
        before = reference.copy()
        sampled = actual_view_reference(reference)
        self.assertTrue(all(any(np.array_equal(point, old) for old in before) for point in sampled))
        np.testing.assert_array_equal(reference, before)
        self.assertLess(len(sampled), len(reference))

    def test_actual_view_reference_keeps_nearest_and_wraps_azimuth(self):
        reference = np.array([[2., 0., 0.], [1., 0., 0.], [-2., 0., 0.], [-1., 0., 0.]])
        sampled = actual_view_reference(reference)
        np.testing.assert_array_equal(sampled, reference[[1, 3]])


if __name__ == '__main__':
    unittest.main()
