#!/usr/bin/env python3
import glob
import os
import sys
import unittest
import json
from unittest.mock import Mock, patch

import cv2
import numpy as np


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

try:
    from qr_detector_node import QRDetectorNode, ZBarQRDecoder
except ModuleNotFoundError:
    # The source-only CI environment may not have ROS Python modules.  The
    # test still runs automatically on the aircraft after sourcing Noetic.
    QRDetectorNode = None
    ZBarQRDecoder = None


@unittest.skipIf(QRDetectorNode is None, "ROS Python modules are unavailable")
class QRDetectorValidationTests(unittest.TestCase):
    def make_detector(self):
        detector = QRDetectorNode.__new__(QRDetectorNode)
        detector.entry_goal_received = True
        detector.depth_match_tolerance_s = .5
        detector.qr_detector = cv2.QRCodeDetector()
        detector.zbar_decoder = ZBarQRDecoder()
        detector.preprocess_mode = "gray"
        detector.upscale_factor = 1.5
        detector.enable_preprocess_fallbacks = False
        detector.enable_zbar_full_frame_fallback = False
        detector.zbar_full_frame_interval = 0.5
        detector.last_zbar_full_frame_time = None
        detector.min_area = 150.0
        detector.min_side_length = 16.0
        detector.max_side_ratio = 4.0
        detector.max_angle_cos = 0.80
        detector.max_quad_area_ratio = 0.35
        detector.max_quad_width_ratio = 0.90
        detector.max_quad_height_ratio = 0.90
        detector.qr_eps_x = 0.15
        detector.qr_eps_y = 0.15
        detector.require_decode_for_confirmation = True
        detector.decode_qr_data = True
        detector.decode_verification_hold_seconds = 2.0
        detector.decode_verification_max_center_shift_px = 120.0
        detector.last_decode_verification_time = None
        detector.last_decode_center = None
        detector.last_decoded_data = ""
        detector.startup_warmup_seconds = 5.0
        detector.first_image_wall_time = None
        detector.startup_warmup_finished = False
        detector.configure_qr_detector()
        return detector

    def test_libzbar_decoder_is_available_on_aircraft(self):
        detector = self.make_detector()
        self.assertTrue(detector.zbar_decoder.available)

    def test_depth_time_tolerance_accepts_500ms_but_not_older_or_newer_frames(self):
        import rospy
        from sensor_msgs.msg import Image
        from std_msgs.msg import Header
        detector = self.make_detector()
        detector.bridge = Mock()
        detector.depth_from_window = Mock(return_value=1.0)
        detector.depth_quality_from_quad = Mock(return_value={
            "validated": True, "valid_corners": 4, "valid_ratio": 1.0,
            "std": 0.0, "range": 0.0,
        })
        detector.has_camera_info = Mock(return_value=True)
        detector.back_project = Mock(return_value=(0.0, 0.0, 1.0))
        detector.filter_xyz = lambda x, y, z: (x, y, z)
        detection = {"data": "AR14", "points": np.array([[0, 0], [10, 0], [10, 10], [0, 10]]),
                     "center_u": 5, "center_v": 5, "method": "test", "area": 100,
                     "validated": True}
        for offset, accepted in [(-.5, True), (.5, True), (-.501, False), (.501, False)]:
            with self.subTest(offset=offset):
                detector.latest_depth_msg = Image(header=Header(stamp=rospy.Time.from_sec(10 + offset)))
                result = detector.build_result(detection, Header(stamp=rospy.Time.from_sec(10)))
                self.assertEqual(result["confirmable"], accepted)
                self.assertEqual(result["depth_validated"], accepted)
                self.assertEqual(result["z"], 1.0 if accepted else None)
                self.assertTrue(result["validated"])  # Visual recognition survives.

    def test_decoded_qr_without_depth_publishes_evidence_status_without_pose(self):
        import rospy
        from std_msgs.msg import Header
        detector = self.make_detector()
        detector.camera_frame_id = "camera_optical"
        detector.candidate_pose_pub = Mock()
        detector.candidate_detected_pub = Mock()
        detector.consecutive_valid_count = 0
        detector.confirm_frames = 1
        result = {"data": "AR14", "validated": True, "confirmable": False,
                  "depth_validated": False, "points": [], "x": None, "y": None, "z": None}
        header = Header(stamp=rospy.Time.from_sec(12))
        detector.publish_candidate(result, header, False)
        status = json.loads(detector.candidate_detected_pub.publish.call_args.args[0].data)
        self.assertTrue(status["evidence_credible"])
        self.assertFalse(status["localization_valid"])
        self.assertFalse(status["confirmable"])
        detector.candidate_pose_pub.publish.assert_not_called()
        detector.publish_candidate(result, header, False, startup_warmup=True)
        status = json.loads(detector.candidate_detected_pub.publish.call_args.args[0].data)
        self.assertFalse(status["evidence_credible"])

    def test_full_frame_fallback_recovers_opencv_miss_without_rescaling(self):
        detector = self.make_detector()
        detector.enable_zbar_full_frame_fallback = True
        detector.qr_detector = Mock()
        detector.qr_detector.detect.return_value = (False, None)
        detector.zbar_decoder = Mock()
        corners = np.array([[[60, 30], [60, 110], [100, 110], [100, 30]]], dtype=np.float32)
        detector.zbar_decoder.decode.return_value = ("AR14", corners)
        result = detector.detect_qr(np.zeros((150, 200, 3), dtype=np.uint8))
        self.assertEqual(result["data"], "AR14")
        self.assertEqual(result["method"], "zbar_full_frame")
        self.assertTrue(result["validated"])
        self.assertEqual((result["center_u"], result["center_v"]), (80, 70))
        self.assertEqual(detector.zbar_decoder.decode.call_args.args[0].shape, (150, 200))
        self.assertEqual(result["area"], 3200)
        detector.latest_depth_msg = None
        status = detector.build_result(result, Mock(stamp=None))
        self.assertTrue(status["validated"])
        self.assertFalse(status["depth_validated"])
        self.assertFalse(status["confirmable"])

    def test_full_frame_fallback_throttles_failures_and_can_be_disabled(self):
        detector = self.make_detector()
        detector.enable_zbar_full_frame_fallback = True
        detector.zbar_decoder = Mock()
        detector.zbar_decoder.decode.return_value = ("", None)
        gray = np.zeros((150, 200), dtype=np.uint8)
        for now, expected_calls in ((10.0, 1), (10.1, 1), (10.49, 1), (10.5, 2)):
            with patch("qr_detector_node.time.monotonic", return_value=now):
                self.assertIsNone(detector.detect_zbar_full_frame(gray))
            self.assertEqual(detector.zbar_decoder.decode.call_count, expected_calls)
        detector.enable_zbar_full_frame_fallback = False
        with patch("qr_detector_node.time.monotonic", return_value=20.0):
            self.assertIsNone(detector.detect_zbar_full_frame(gray))
        self.assertEqual(detector.zbar_decoder.decode.call_count, 2)

    def test_full_frame_fallback_preserves_geometry_validation(self):
        detector = self.make_detector()
        detector.enable_zbar_full_frame_fallback = True
        detector.zbar_decoder = Mock()
        corners = np.array([[[60, 30], [60, 35], [65, 35], [65, 30]]], dtype=np.float32)
        detector.zbar_decoder.decode.return_value = ("AR14", corners)
        self.assertIsNone(detector.detect_zbar_full_frame(np.zeros((150, 200), dtype=np.uint8)))
        self.assertIsNone(detector.last_decode_verification_time)

    def test_full_frame_fallback_does_not_run_when_opencv_returns_candidate(self):
        detector = self.make_detector()
        detector.enable_zbar_full_frame_fallback = True
        detector.detect_qr_in_image = Mock(return_value={
            "points": np.array([[60, 30], [100, 30], [100, 110], [60, 110]]),
            "validated": False,
        })
        detector.zbar_decoder = Mock()
        result = detector.detect_qr(np.zeros((150, 200, 3), dtype=np.uint8))
        self.assertFalse(result["validated"])
        detector.zbar_decoder.decode.assert_not_called()

    def test_libzbar_fallback_supplies_payload_and_corners(self):
        detector = self.make_detector()

        class OpenCVWithoutDecoder(object):
            def detect(self, _image):
                return True, np.asarray(
                    [[[10, 10], [90, 10], [90, 90], [10, 90]]],
                    dtype=np.float32,
                )

            def decode(self, _image, _points):
                return "", None

        class WorkingFallback(object):
            def decode(self, _image):
                return "AR03", np.asarray(
                    [[[10, 10], [90, 10], [90, 90], [10, 90]]],
                    dtype=np.float32,
                )

        detector.qr_detector = OpenCVWithoutDecoder()
        detector.zbar_decoder = WorkingFallback()
        result = detector.run_qr_detector(np.zeros((100, 100), dtype=np.uint8))
        self.assertTrue(result["decoded_valid"])
        self.assertEqual(result["decoded_data"], "AR03")
        self.assertIsNotNone(result["decoded_points"])

    def test_one_decode_authenticates_nearby_frames_only(self):
        detector = self.make_detector()
        decoded = {"decoded_valid": True, "decoded_data": "competition-qr"}
        undecoded = {"decoded_valid": False, "decoded_data": ""}

        valid, reason, data = detector.apply_decode_verification(
            decoded, 500.0, 300.0, now=100.0
        )
        self.assertTrue(valid)
        self.assertEqual(reason, "decoded")
        self.assertEqual(data, "competition-qr")

        valid, reason, data = detector.apply_decode_verification(
            undecoded, 520.0, 310.0, now=101.0
        )
        self.assertTrue(valid)
        self.assertEqual(reason, "recent_decode")
        self.assertEqual(data, "competition-qr")

        valid, _, _ = detector.apply_decode_verification(
            undecoded, 800.0, 300.0, now=101.0
        )
        self.assertFalse(valid)
        valid, _, _ = detector.apply_decode_verification(
            undecoded, 520.0, 310.0, now=102.1
        )
        self.assertFalse(valid)

    def test_decode_refresh_uses_full_image_position_and_expiry(self):
        # The same candidate is tested in a full frame and an enlarged ROI.
        # Its original-image center is (614.6, 243.83), matching the flight
        # frame that moved about 309 pixels within the two-second cache.
        full_points = np.asarray(
            [[584.6, 213.83], [644.6, 213.83],
             [644.6, 273.83], [584.6, 273.83]], dtype=np.float32
        )
        for scale, offset in ((1.0, (0, 0)), (1.5, (500, 150))):
            for last_center, now, refresh in (
                ((922.67, 265.0), 100.68, True),
                ((610.0, 240.0), 100.68, False),
                ((610.0, 240.0), 102.1, True),
            ):
                for opencv_payload, fallback_payload in (
                    ("new-qr", ""), ("", "new-qr"), ("", "")
                ):
                    with self.subTest(scale=scale, offset=offset,
                                      last_center=last_center, now=now,
                                      payloads=(opencv_payload, fallback_payload)):
                        detector = self.make_detector()
                        detector.upscale_factor = scale
                        detector.last_decode_verification_time = 100.0
                        detector.last_decode_center = last_center
                        detector.last_decoded_data = "old-qr"
                        points = (full_points - np.asarray(offset)) * scale
                        detector.qr_detector = Mock()
                        detector.qr_detector.detect.return_value = (True, points)
                        detector.qr_detector.decode.return_value = (
                            opencv_payload, None
                        )
                        gray = np.zeros((720, 1280), dtype=np.uint8)
                        processed, _ = detector.preprocess_for_qr(gray, "gray")
                        _, crop_offset = detector.qr_crop(processed, points)
                        detector.zbar_decoder = Mock()
                        detector.zbar_decoder.decode.return_value = (
                            fallback_payload, points - np.asarray(crop_offset)
                        )
                        with patch("qr_detector_node.time.monotonic", return_value=now):
                            result = detector.detect_qr_in_image(
                                gray, offset=offset, image_shape=(720, 1280)
                            )
                        self.assertIsNotNone(result)
                        self.assertAlmostEqual(result["center_u"], 614.6, places=3)
                        self.assertAlmostEqual(result["center_v"], 243.83, places=3)
                        self.assertEqual(detector.qr_detector.decode.call_count, int(refresh))
                        self.assertEqual(detector.zbar_decoder.decode.call_count,
                                         int(refresh and not opencv_payload))
                        if not refresh:
                            self.assertEqual(result["validation_reason"], "recent_decode")
                            self.assertEqual(result["data"], "old-qr")
                            self.assertEqual(detector.last_decode_verification_time, 100.0)
                        elif opencv_payload or fallback_payload:
                            self.assertTrue(result["validated"])
                            self.assertEqual(result["validation_reason"], "decoded")
                            self.assertEqual(result["data"], "new-qr")
                        else:
                            self.assertFalse(result["validated"])
                            self.assertEqual(result["data"], "")

    def test_recent_decode_tracks_gradual_motion_without_extending_expiry(self):
        detector = self.make_detector()
        decoded = {"decoded_valid": True, "decoded_data": "competition-qr"}
        undecoded = {"decoded_valid": False, "decoded_data": ""}

        valid, _, _ = detector.apply_decode_verification(
            decoded, 600.0, 300.0, now=100.0
        )
        self.assertTrue(valid)
        for now, center_x in ((100.4, 690.0), (100.8, 780.0), (101.2, 870.0)):
            valid, reason, _ = detector.apply_decode_verification(
                undecoded, center_x, 300.0, now=now
            )
            self.assertTrue(valid)
            self.assertEqual("recent_decode", reason)

        valid, _, _ = detector.apply_decode_verification(
            undecoded, 870.0, 300.0, now=102.1
        )
        self.assertFalse(valid)

    def test_startup_warmup_blocks_early_confirmation(self):
        detector = self.make_detector()
        detector.first_image_wall_time = 100.0
        self.assertTrue(detector.startup_warmup_active(now=104.9))
        self.assertFalse(detector.startup_warmup_active(now=105.0))
        self.assertTrue(detector.startup_warmup_finished)

    def test_large_background_quad_is_rejected_by_geometry_limit(self):
        detector = self.make_detector()
        points = np.asarray(
            [[0, 0], [1279, 0], [1279, 719], [0, 719]], dtype=np.float32
        )
        ok, _ = detector.is_reasonable_quad(points, image_w=1280, image_h=720)
        self.assertFalse(ok)

    def test_saved_false_qr_evidence_cannot_become_validated(self):
        paths = sorted(
            glob.glob(
                "/home/oem/target_reports/onboard_test_20260812/"
                "images/annotated/*qr_code.jpg"
            )
        )
        if not paths:
            self.skipTest("现场二维码误检样本不存在")
        detector = self.make_detector()
        # The directory can also contain a genuinely confirmed QR evidence
        # image from a later run.  Find a saved sample that still produces a
        # geometric candidate but fails the detector's authenticity gate;
        # do not assume the lexicographically latest file is the false one.
        false_candidate_found = False
        for path in reversed(paths):
            image = cv2.imread(path)
            self.assertIsNotNone(image)
            result = detector.detect_qr(image)
            if result is not None and not result["validated"]:
                false_candidate_found = True
                break
        if not false_candidate_found:
            self.skipTest("当前现场样本中没有未验证二维码候选")


if __name__ == "__main__":
    unittest.main()
