#!/usr/bin/env python3
import json
import os
import sys
import unittest
from unittest.mock import patch

import rospy

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
    def setUp(self):
        clock = patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(13))
        clock.start()
        self.addCleanup(clock.stop)

    def make_node(self):
        node = ThermalD435FusionNode.__new__(ThermalD435FusionNode)
        node.report_candidate_pub = RecordingPublisher()

        def build_camera_point(
            thermal_point, _info, _depth, _encoding, _thermal_bbox=None
        ):
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

    def test_report_candidates_passes_each_bbox_to_depth_fusion(self):
        node = self.make_node()
        received_bboxes = []
        original = node.build_camera_point

        def recording_build_camera_point(
            thermal_point, info, depth, encoding, thermal_bbox=None
        ):
            received_bboxes.append(thermal_bbox)
            return original(thermal_point, info, depth, encoding, thermal_bbox)

        node.build_camera_point = recording_build_camera_point
        status = {
            "stamp": 12.5,
            "candidates": [
                {"cx": 100, "cy": 80, "bbox": [90, 70, 20, 20]},
                {"cx": 220, "cy": 120, "bbox": [210, 110, 20, 20]},
            ],
        }

        self.assertTrue(
            node.publish_report_candidates(status, CameraInfo(), object(), "16UC1")
        )
        self.assertEqual(
            [[90, 70, 20, 20], [210, 110, 20, 20]], received_bboxes
        )

    def test_legacy_single_candidate_status_remains_supported(self):
        status = {"detected": True, "cx": 100, "cy": 80}
        self.assertEqual([status], ThermalD435FusionNode.status_candidates(status))

    def test_missing_depth_retains_stable_thermal_evidence_and_camera_ray(self):
        node = self.make_node()
        node.build_camera_point = lambda *args: None
        node.d435_optical_frame = "camera_optical"
        node.map_thermal_to_d435 = lambda x, y: (x, y)
        info = CameraInfo()
        info.K = [200, 0, 100, 0, 200, 80, 0, 0, 1]
        status = {"stamp": 12.5, "candidates": [
            {"cx": 120, "cy": 80, "stable": True},
            {"cx": 220, "cy": 120, "stable": False},
        ]}
        self.assertTrue(node.publish_report_candidates(status, info, None, None))
        decoded = [json.loads(m.data) for m in node.report_candidate_pub.messages]
        self.assertEqual([True, False], [d["evidence_credible"] for d in decoded])
        self.assertTrue(all(d["camera_point"] is None for d in decoded))
        self.assertTrue(all(not d["confirmable"] for d in decoded))
        self.assertAlmostEqual(decoded[0]["camera_ray"]["x"], 0.1)
        self.assertEqual(decoded[0]["camera_ray"]["z"], 1.0)

    def test_missing_camera_info_still_retains_thermal_image_evidence(self):
        node = self.make_node()
        node.build_camera_point = lambda *args: None
        status = {"stamp": 12.5, "detected": True, "cx": 100, "cy": 80, "stable": True}
        node.publish_report_candidates(status, None, None, None)
        data = json.loads(node.report_candidate_pub.messages[0].data)
        self.assertTrue(data["evidence_credible"])
        self.assertIsNone(data["camera_ray"])


if __name__ == "__main__":
    unittest.main()
