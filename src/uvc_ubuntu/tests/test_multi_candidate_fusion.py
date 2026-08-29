#!/usr/bin/env python3
import json
import os
import sys
import unittest

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from thermal_d435_fusion_node import ThermalD435FusionNode


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class MultiCandidateFusionTests(unittest.TestCase):
    def make_node(self):
        node = ThermalD435FusionNode.__new__(ThermalD435FusionNode)
        node.report_candidate_pub = RecordingPublisher()

        def build_camera_point(thermal_point, _info, _depth, _encoding):
            point = PointStamped()
            point.header.frame_id = "camera_optical"
            point.point.x = thermal_point.point.x / 100.0
            point.point.y = thermal_point.point.y / 100.0
            point.point.z = 1.0
            return point

        node.build_camera_point = build_camera_point
        return node

    def test_publishes_each_candidate_with_its_camera_point(self):
        node = self.make_node()
        status = {
            "stamp": 12.5,
            "stable_window": 3,
            "candidates": [
                {
                    "rank": 1,
                    "cx": 100,
                    "cy": 80,
                    "bbox": [90, 70, 20, 20],
                    "stable": True,
                    "stable_count": 2,
                },
                {
                    "rank": 2,
                    "cx": 220,
                    "cy": 120,
                    "bbox": [210, 110, 20, 20],
                    "stable": False,
                    "stable_count": 1,
                },
            ],
        }

        self.assertTrue(
            node.publish_report_candidates(status, CameraInfo(), object(), "16UC1")
        )
        self.assertEqual(2, len(node.report_candidate_pub.messages))

        decoded = [
            json.loads(message.data)
            for message in node.report_candidate_pub.messages
        ]
        self.assertNotEqual(decoded[0]["stamp"], decoded[1]["stamp"])
        self.assertEqual([True, False], [item["stable"] for item in decoded])
        self.assertEqual([1, 2], [item["rank"] for item in decoded])
        self.assertEqual(
            [1.0, 2.2], [item["camera_point"]["x"] for item in decoded]
        )

    def test_legacy_single_candidate_status_remains_supported(self):
        status = {"detected": True, "cx": 100, "cy": 80}
        self.assertEqual([status], ThermalD435FusionNode.status_candidates(status))


if __name__ == "__main__":
    unittest.main()
