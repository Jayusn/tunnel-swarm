#!/usr/bin/env python3
import os
import sys
import unittest

import cv2
import numpy as np


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from thermal_detect import detect_hotspot
from thermal_ros_node import MultiTargetFilter


class HotspotDetectorTests(unittest.TestCase):
    def test_complete_hotspot_does_not_touch_roi_border(self):
        image = np.zeros((288, 384), dtype=np.uint8)
        cv2.rectangle(image, (150, 90), (200, 150), 255, -1)
        result = detect_hotspot(image)
        self.assertTrue(result["detected"])
        self.assertFalse(result["touches_roi_border"])

    def test_clipped_hotspot_is_marked_as_roi_border_touching(self):
        image = np.zeros((288, 384), dtype=np.uint8)
        # The detector ROI ends at x=354, so this hotspot is clipped by it.
        cv2.rectangle(image, (340, 90), (370, 150), 255, -1)
        result = detect_hotspot(image)
        self.assertTrue(result["detected"])
        self.assertTrue(result["touches_roi_border"])

    def test_brightest_valid_hotspot_remains_selected(self):
        image = np.zeros((288, 384), dtype=np.uint8)
        cv2.rectangle(image, (90, 90), (130, 130), 180, -1)
        cv2.rectangle(image, (220, 90), (260, 130), 255, -1)
        result = detect_hotspot(image)
        self.assertTrue(result["detected"])
        self.assertGreater(result["cx"], 200)

    def test_returns_multiple_hotspots_in_brightness_order(self):
        image = np.zeros((288, 384), dtype=np.uint8)
        cv2.rectangle(image, (90, 90), (130, 130), 180, -1)
        cv2.rectangle(image, (220, 90), (260, 130), 255, -1)
        result = detect_hotspot(image)
        self.assertEqual(2, len(result["candidates"]))
        self.assertGreater(
            result["candidates"][0]["mean_gray"],
            result["candidates"][1]["mean_gray"],
        )

    def test_limits_number_of_returned_hotspots(self):
        image = np.zeros((288, 384), dtype=np.uint8)
        for x in (60, 120, 180, 240):
            cv2.rectangle(image, (x, 90), (x + 20, 110), 255, -1)
        result = detect_hotspot(image, max_candidates=2)
        self.assertEqual(2, len(result["candidates"]))


class MultiTargetFilterTests(unittest.TestCase):
    def test_tracks_each_current_hotspot_across_frames(self):
        stability = MultiTargetFilter(max_len=3, min_hits=2, max_pixel_jump=5.0)
        first = stability.update(
            [{"cx": 50, "cy": 60}, {"cx": 200, "cy": 100}]
        )
        second = stability.update(
            [{"cx": 52, "cy": 61}, {"cx": 198, "cy": 102}]
        )
        self.assertEqual([False, False], [item["stable"] for item in first])
        self.assertEqual([True, True], [item["stable"] for item in second])

    def test_does_not_confirm_new_hotspot_from_other_stable_track(self):
        stability = MultiTargetFilter(max_len=3, min_hits=2, max_pixel_jump=5.0)
        stability.update([{"cx": 50, "cy": 60}])
        result = stability.update(
            [{"cx": 51, "cy": 60}, {"cx": 200, "cy": 100}]
        )
        self.assertEqual([True, False], [item["stable"] for item in result])


if __name__ == "__main__":
    unittest.main()
