#!/usr/bin/env python3
import os
import sys
import unittest

import numpy as np


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from thermal_d435_fusion_node import ThermalD435FusionNode


class DepthBBoxFallbackTests(unittest.TestCase):
    def make_node(self):
        node = ThermalD435FusionNode.__new__(ThermalD435FusionNode)
        node.homography = np.eye(3, dtype=np.float64)
        node.depth_scale = 0.001
        node.depth_search_radius = 2
        node.min_valid_depth = 0.2
        node.max_valid_depth = 5.0
        node.depth_encoding = "16UC1"
        return node

    def test_center_depth_remains_preferred(self):
        node = self.make_node()
        depth = np.full((80, 80), 1000, dtype=np.uint16)
        depth[38:43, 38:43] = 2000

        selected, source = node.lookup_target_depth(
            depth, 40, 40, [10, 10, 60, 60], "16UC1"
        )

        self.assertAlmostEqual(2.0, selected, places=3)
        self.assertEqual("center", source)

    def test_bbox_uses_nearest_meaningful_foreground_cluster(self):
        node = self.make_node()
        depth = np.full((100, 100), 3000, dtype=np.uint16)
        depth[45:56, 45:56] = 0
        depth[20:40, 20:40] = 1100

        selected, source = node.lookup_target_depth(
            depth, 50, 50, [10, 10, 80, 80], "16UC1"
        )

        self.assertAlmostEqual(1.1, selected, places=3)
        self.assertEqual("bbox", source)

    def test_sparse_near_noise_does_not_replace_dominant_depth(self):
        node = self.make_node()
        depth = np.full((80, 80), 2800, dtype=np.uint16)
        depth[38:43, 38:43] = 0
        depth[10, 10] = 500

        selected, source = node.lookup_target_depth(
            depth, 40, 40, [5, 5, 70, 70], "16UC1"
        )

        self.assertAlmostEqual(2.8, selected, places=3)
        self.assertEqual("bbox", source)

    def test_returns_invalid_when_bbox_has_no_depth(self):
        node = self.make_node()
        depth = np.zeros((80, 80), dtype=np.uint16)

        selected, source = node.lookup_target_depth(
            depth, 40, 40, [5, 5, 70, 70], "16UC1"
        )

        self.assertIsNone(selected)
        self.assertIsNone(source)


if __name__ == "__main__":
    unittest.main()
