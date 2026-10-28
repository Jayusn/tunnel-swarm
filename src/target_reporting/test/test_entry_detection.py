"""Exercise the three image pipelines before and after the entry goal."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped


ROOT = Path(__file__).resolve().parents[2]


def load(relative):
    spec = importlib.util.spec_from_file_location(Path(relative).stem, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RecognitionStarted(Exception):
    pass


def test_color_keeps_preview_but_does_not_recognize_until_entry():
    module = load("color_tag_detector/scripts/color_tag_detector.py")
    node = module.ColorTagDetector.__new__(module.ColorTagDetector)
    node.entry_goal_received = False
    node.bridge = CvBridge()
    node.colors = {"red": {}}
    node.build_color_mask = Mock(side_effect=RecognitionStarted)
    for name in ("publish_debug", "publish_mask", "publish_no_detection", "publish_result",
                 "publish_unlocalized_evidence", "update_stability"):
        setattr(node, name, Mock())
    color = node.bridge.cv2_to_imgmsg(np.zeros((80, 160, 3), np.uint8), "bgr8")
    depth = node.bridge.cv2_to_imgmsg(np.zeros((80, 160), np.uint16), "16UC1")
    node.image_callback(color, depth)
    node.publish_debug.assert_called_once()
    node.build_color_mask.assert_not_called()
    node.publish_result.assert_not_called()
    node.publish_unlocalized_evidence.assert_not_called()
    node.update_stability.assert_not_called()
    node.entry_goal_callback(PoseStamped())
    with pytest.raises(RecognitionStarted):
        node.image_callback(color, depth)


def test_qr_keeps_preview_but_does_not_decode_until_entry():
    module = load("qr_detector/scripts/qr_detector_node.py")
    node = module.QRDetectorNode.__new__(module.QRDetectorNode)
    node.entry_goal_received = False
    node.bridge = CvBridge()
    node.startup_warmup_active = Mock(return_value=False)
    node.publish_status = Mock()
    node.draw_debug_image = Mock(side_effect=lambda image, *args: image)
    node.publish_debug = Mock()
    node.detect_qr = Mock(side_effect=RecognitionStarted)
    node.publish_candidate = Mock()
    color = node.bridge.cv2_to_imgmsg(np.zeros((80, 160, 3), np.uint8), "bgr8")
    node.image_callback(color)
    node.publish_debug.assert_called_once()
    node.detect_qr.assert_not_called()
    node.publish_candidate.assert_not_called()
    node.entry_goal_callback(PoseStamped())
    with pytest.raises(RecognitionStarted):
        node.image_callback(color)


def test_thermal_drains_camera_and_keeps_preview_until_entry(monkeypatch):
    module = load("uvc_ubuntu/thermal_ros_node.py")
    node = module.ThermalDetectorNode.__new__(module.ThermalDetectorNode)
    for name, value in dict(entry_goal_received=False, publish_rate=0, process=None,
                            frame_size=8, width=2, height=2, shift_x=0, shift_y=0,
                            roi_margin_x=0, roi_margin_y=0, threshold_k=2,
                            min_area=1, max_area=4, max_candidates=1,
                            min_local_contrast=0, display=False).items():
        setattr(node, name, value)
    node.start = Mock()
    node.image_pub = Mock()
    node.debug_pub = Mock()
    node.publish_detection = Mock()
    monkeypatch.setattr(module.rospy.Time, "now", staticmethod(lambda: module.rospy.Time.from_sec(10)))
    monkeypatch.setattr(module.rospy, "is_shutdown", Mock(side_effect=[False, False, True]))
    def read(*args):
        if node.image_pub.publish.called:
            node.entry_goal_callback(PoseStamped())
        return bytes(8)
    monkeypatch.setattr(module, "read_frame", read)
    detect = Mock(return_value={"detected": False})
    monkeypatch.setattr(module, "detect_hotspot", detect)
    monkeypatch.setattr(module, "draw_debug", Mock(return_value=np.zeros((2, 2, 3), np.uint8)))
    node.spin()
    node.start.assert_called_once()  # Camera starts before the goal arrives.
    node.image_pub.publish.assert_called_once()  # Waiting preview.
    node.debug_pub.publish.assert_called_once()
    detect.assert_called_once()  # Only the second frame is recognized.
    node.publish_detection.assert_called_once()
