"""Offline tests for restart alignment and inverse flight coordinates."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from saved_map_math import rotation, register, downsample


class SavedMapTest(unittest.TestCase):
    def test_restart_alignment_with_changed_heading_and_position(self):
        rng = np.random.RandomState(12)
        target = rng.uniform([-3, -3, -.5], [4, 4, 2.], (7000, 3))
        # Non-symmetric static walls with smaller random measurement errors.
        target[:2300, 0] = -2.
        target[2300:4600, 1] = 2.5
        yaw, shift = .12, np.array([.17, -.09, .04])
        source = (target-shift) @ rotation(yaw)
        source += rng.normal(0., .003, source.shape)
        found_yaw, found_shift, metrics = register(source, target)
        self.assertAlmostEqual(found_yaw, yaw, delta=.005)
        np.testing.assert_allclose(found_shift, shift, atol=.008)
        self.assertGreater(metrics['inlier_ratio'], .95)
        self.assertLess(metrics['rmse_m'], .04)  # 10cm voxel selection adds quantization error
        # A saved-map goal must arrive at the same physical point in current boot.
        goal = np.array([3., 3., .5])
        local = rotation(found_yaw).T @ (goal-found_shift)
        np.testing.assert_allclose(rotation(found_yaw) @ local + found_shift, goal, atol=1e-12)

    def test_nonoverlapping_cloud_rejected(self):
        rng = np.random.RandomState(3)
        points = rng.rand(1500, 3)
        with self.assertRaises(ValueError):
            register(points+10., points)

    def test_empty_cloud_rejected(self):
        with self.assertRaises(ValueError):
            register([], [])

    def test_downsample_drops_invalid_and_duplicate_voxels(self):
        self.assertEqual(len(downsample([[0, 0, 0], [.01, 0, 0], [np.nan, 0, 0]])), 1)


if __name__ == '__main__':
    unittest.main()
