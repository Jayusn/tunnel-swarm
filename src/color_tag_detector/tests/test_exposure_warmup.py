import json
import sys
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import rospy
from cv_bridge import CvBridge

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import color_tag_detector as module


def make_detector():
    detector = module.ColorTagDetector.__new__(module.ColorTagDetector)
    detector.bridge = CvBridge()
    detector.exposure_warmup_seconds = 5.0
    detector.first_image_wall_time = None
    detector.colors = {}
    detector.update_stability = Mock(return_value={})
    detector.select_best_candidate = Mock(return_value=None)
    detector.draw_debug_image = Mock(side_effect=lambda image, *args: image)
    detector.publish_result = Mock()
    detector.publish_unlocalized_evidence = Mock()
    detector.publish_debug = Mock()
    detector.publish_mask = Mock()
    detector.text_pub = Mock()
    return detector


def images(detector):
    color = detector.bridge.cv2_to_imgmsg(np.zeros((80, 160, 3), np.uint8), "bgr8")
    # Invalid depth is still a usable image: it must not bypass warm-up via
    # the separate unlocalized-evidence path.
    depth = detector.bridge.cv2_to_imgmsg(np.zeros((80, 160), np.uint16), "16UC1")
    color.header.stamp = rospy.Time.from_sec(10)
    return color, depth


def test_warmup_starts_at_first_frame_and_blocks_both_evidence_paths(monkeypatch):
    detector = make_detector()
    clock = [1000.0]  # Long after process startup; still wait from first image.
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(rospy, "loginfo", Mock())
    color, depth = images(detector)
    for elapsed in (0, 2, 4.99):
        clock[0] = 1000 + elapsed
        detector.image_callback(color, depth)
        result = json.loads(detector.text_pub.publish.call_args.args[0].data)
        assert result["detected"] is False
        assert result["reason"] == "exposure_warmup"
    detector.update_stability.assert_not_called()
    detector.publish_result.assert_not_called()
    detector.publish_unlocalized_evidence.assert_not_called()
    assert detector.publish_debug.call_count == 3
    assert detector.first_image_wall_time == 1000
    # Wall-clock wait is independent of ROS image timestamps.
    clock[0] = 1005
    detector.image_callback(color, depth)
    detector.update_stability.assert_called_once()
    detector.publish_result.assert_called_once()
    detector.publish_unlocalized_evidence.assert_called_once()
    clock[0] = 1010
    detector.image_callback(color, depth)
    assert detector.publish_result.call_count == 2


def test_zero_disables_wait():
    detector = make_detector()
    detector.exposure_warmup_seconds = 0
    detector.image_callback(*images(detector))
    detector.publish_result.assert_called_once()
    detector.publish_unlocalized_evidence.assert_called_once()
    detector.text_pub.publish.assert_not_called()


def test_missing_image_does_not_start_warmup():
    detector = make_detector()
    color, depth = images(detector)
    detector.bridge.imgmsg_to_cv2 = Mock(return_value=None)
    detector.image_callback(color, depth)
    assert detector.first_image_wall_time is None
    detector.publish_result.assert_not_called()
    detector.publish_unlocalized_evidence.assert_not_called()
