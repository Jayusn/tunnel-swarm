import io
import json
import struct
import sys
import threading
from collections import deque
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from rospy.msg import deserialize_messages
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import thermal_ros_node as detector
from thermal_d435_fusion_node import ThermalD435FusionNode


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def stamp(seconds):
    return rospy.Time.from_sec(seconds)


def make_node(monkeypatch):
    monkeypatch.setattr(rospy.Time, "now", staticmethod(lambda: stamp(20)))
    monkeypatch.setattr(rospy, "loginfo_throttle", lambda *args: None)
    node = ThermalD435FusionNode.__new__(ThermalD435FusionNode)
    node.lock = threading.Lock()
    node.bridge = CvBridge()
    node.depth_frames = deque(maxlen=10)
    node.color_frames = deque(maxlen=10)
    node.fusion_delay_s = 0
    node.pending_candidate_statuses = deque(maxlen=100)
    node.fusion_candidate_status = None
    node.fusion_candidate_status_version = 0
    node.candidate_status = None
    node.candidate_status_version = 0
    node.published_candidate_status_version = 0
    node.target_detected = False
    node.thermal_pixel_msg = None
    node.candidate_target_detected = False
    node.candidate_thermal_pixel_msg = None
    node.candidate_thermal_bbox = None
    node.first_color_wall_time = None
    node.d435_exposure_warmup_seconds = 0
    node.homography = np.eye(3)
    node.depth_scale = .001
    node.depth_search_radius = 0
    node.min_valid_depth = .2
    node.max_valid_depth = 5
    node.d435_optical_frame = "camera_optical"
    node.camera_info = CameraInfo(K=[100, 0, 2, 0, 100, 2, 0, 0, 1])
    node.thermal_ground_filter_enabled = False
    node.use_tf = False
    node.debug = False
    for name in ("report_candidate_pub", "d435_evidence_image_pub", "d435_debug_image_pub",
                 "candidate_camera_point_pub", "d435_pixel_pub", "camera_point_pub",
                 "channel_position_pub", "fusion_valid_pub"):
        setattr(node, name, Publisher())
    return node


def add_frame(node, seconds, depth_mm, color_value):
    depth = node.bridge.cv2_to_imgmsg(np.full((20, 20), depth_mm, np.uint16), "16UC1")
    depth.header.stamp = stamp(seconds)
    color = node.bridge.cv2_to_imgmsg(np.full((20, 20, 3), color_value, np.uint8), "bgr8")
    color.header.stamp = stamp(seconds + .002)
    color.header.frame_id = "camera_optical"
    node.depth_callback(depth)
    node.color_callback(color)
    return depth, color


def test_delayed_candidate_uses_nearest_received_frames_and_preserves_headers(monkeypatch):
    node = make_node(monkeypatch)
    _, original_color = add_frame(node, 10.01, 1000, 11)
    add_frame(node, 10.20, 3000, 99)
    # An older frame arriving last must not win just because of arrival order.
    add_frame(node, 9.90, 2000, 55)
    node.target_detected = False
    node.thermal_pixel_msg = None
    node.candidate_target_detected = True
    node.candidate_thermal_pixel_msg = PointStamped()
    node.candidate_thermal_pixel_msg.header.stamp = stamp(10)
    node.candidate_thermal_pixel_msg.point.x = 2
    node.candidate_thermal_pixel_msg.point.y = 2
    node.candidate_thermal_pixel_msg.point.z = 1
    node.candidate_thermal_bbox = [1, 1, 4, 4]
    node.candidate_status = {"stamp": 10, "candidates": [
        {"cx": 2, "cy": 2, "stable": True}, {"cx": 4, "cy": 4, "stable": True}]}
    node.candidate_status_version = 1
    node.published_candidate_status_version = 0
    node.timer_callback(None)
    results = [json.loads(msg.data) for msg in node.report_candidate_pub.messages]
    assert len(results) == 2
    assert results[0]["stamp"] != results[1]["stamp"]
    for result in results:
        assert result["camera_point"]["z"] == pytest.approx(1.0)
        timing = result["source_timing"]
        assert timing["thermal_stamp"] == 10
        assert timing["color_dt_s"] == pytest.approx(.012)
        assert timing["depth_dt_s"] == pytest.approx(.010)
    assert original_color.header.stamp == stamp(10.012)
    assert node.color_frames[0][0] == node.color_frames[0][2].stamp
    assert node.camera_point_pub.messages[0].header.stamp == stamp(10)
    assert node.candidate_camera_point_pub.messages[0].point.z == pytest.approx(1)
    for event, msg in zip(results, node.d435_evidence_image_pub.messages):
        assert msg.header.stamp == stamp(event["stamp"])
        # Bottom left is outside all overlay marks in this small test image.
        image = node.bridge.imgmsg_to_cv2(msg, "bgr8")
        assert image[-1, 0, 0] == 11
    assert node.d435_debug_image_pub.messages[0].header.stamp == stamp(10.012)


def test_cache_selects_each_stream_independently_without_new_age_gate(monkeypatch):
    node = make_node(monkeypatch)
    assert node.matched_d435_frames(stamp(10)) == (None, None, None, None, None)
    add_frame(node, 9, 1000, 10)
    color_msg = node.bridge.cv2_to_imgmsg(np.zeros((20, 20, 3), np.uint8), "bgr8")
    color_msg.header.stamp = stamp(10.03)
    node.color_callback(color_msg)
    depth, encoding, depth_time, color, header = node.matched_d435_frames(stamp(10))
    assert depth_time == stamp(9)  # Existing behavior retained even for large gaps.
    assert depth[0, 0] == 1000 and encoding == "16UC1"
    assert header.stamp == stamp(10.03)
    for i in range(20):
        add_frame(node, 20 + i, 1000, i)
    assert len(node.depth_frames) == len(node.color_frames) == 10


def test_receive_callbacks_defer_conversion_until_selection(monkeypatch):
    node = make_node(monkeypatch)
    convert = Mock(wraps=node.bridge.imgmsg_to_cv2)
    monkeypatch.setattr(node.bridge, "imgmsg_to_cv2", convert)
    add_frame(node, 9.8, 2000, 22)
    depth_msg, color_msg = add_frame(node, 10.01, 1000, 11)
    add_frame(node, 10.2, 3000, 33)
    assert convert.call_count == 0
    node.matched_d435_frames(stamp(10))
    assert convert.call_count == 2
    assert convert.call_args_list[0].args[0] is depth_msg
    assert convert.call_args_list[1].args[0] is color_msg


def test_receive_burst_retains_frame_closest_to_thermal_time(monkeypatch):
    # Exercise rospy's actual transport deserializer: queue_size=1 drops the
    # useful earlier frames before the fusion cache can ever receive them.
    errors = []
    for queue_size in (1, 5):
        node = make_node(monkeypatch)
        wire = io.BytesIO()
        for seconds in (10.0, 10.033, 10.067, 10.1, 10.133):
            msg = node.bridge.cv2_to_imgmsg(np.full((20, 20), 1000, np.uint16), "16UC1")
            msg.header.stamp = stamp(seconds)
            payload = io.BytesIO()
            msg.serialize(payload)
            wire.write(struct.pack("<I", len(payload.getvalue())))
            wire.write(payload.getvalue())
        messages = []
        deserialize_messages(wire, messages, Image, queue_size=queue_size)
        for msg in messages:
            node.depth_callback(msg)
        _, _, depth_time, _, _ = node.matched_d435_frames(stamp(10.01))
        errors.append(abs((depth_time - stamp(10.01)).to_sec()))
    assert errors == pytest.approx([.123, .010])


def test_candidate_and_report_keep_same_depth_when_frame_arrives_during_drawing(monkeypatch):
    node = make_node(monkeypatch)
    node.fusion_delay_s = .4
    add_frame(node, 9.98, 1000, 11)
    send_candidate(node, 10)
    # A closer frame appears after the report's snapshot was selected.
    monkeypatch.setattr(node, "publish_d435_debug_image",
                        lambda *args, **kwargs: add_frame(node, 10, 2000, 22))
    node.timer_callback(None)
    report = json.loads(node.report_candidate_pub.messages[0].data)
    assert report["camera_point"]["z"] == pytest.approx(1)
    assert node.candidate_camera_point_pub.messages[0].point.z == pytest.approx(1)


def send_candidate(node, seconds, cx=2):
    status = {"stamp": seconds, "detected": cx is not None,
              "candidates": [] if cx is None else [
                  {"cx": cx, "cy": 2, "bbox": [1, 1, 4, 4], "stable": True}]}
    node.candidate_status_callback(String(data=json.dumps(status)))


def test_delay_pairs_late_images_with_original_event_not_latest_detection(monkeypatch):
    node = make_node(monkeypatch)
    node.fusion_delay_s = .4
    clock = [10.0]
    monkeypatch.setattr(rospy.Time, "now", staticmethod(lambda: stamp(clock[0])))
    add_frame(node, 9.8, 3000, 99)
    send_candidate(node, 10.0)
    node.timer_callback(None)
    assert not node.report_candidate_pub.messages
    # A new detection arrives before the images for the old detection.
    clock[0] = 10.2
    send_candidate(node, 10.2, cx=8)
    clock[0] = 10.35
    _, original_color = add_frame(node, 10.01, 1000, 11)
    node.timer_callback(None)
    assert not node.report_candidate_pub.messages
    clock[0] = 10.401
    node.timer_callback(None)
    result = json.loads(node.report_candidate_pub.messages[0].data)
    assert result["stamp"] == 10
    assert result["cx"] == 2
    assert result["camera_point"]["z"] == pytest.approx(1)
    assert result["source_timing"]["depth_dt_s"] == pytest.approx(.01)
    assert result["source_timing"]["color_dt_s"] == pytest.approx(.012)
    assert result["source_timing"]["fusion_age_s"] == pytest.approx(.401)
    assert result["source_timing"]["color_depth_dt_s"] == pytest.approx(.002)
    assert original_color.header.stamp == stamp(10.012)
    assert node.d435_evidence_image_pub.messages[0].header.stamp == stamp(10)
    assert node.camera_point_pub.messages[0].header.stamp == stamp(10)
    assert node.candidate_camera_point_pub.messages[0].header.stamp == stamp(10)
    assert node.candidate_camera_point_pub.messages[0].point.x == pytest.approx(0)
    node.timer_callback(None)
    assert len(node.report_candidate_pub.messages) == 1
    clock[0] = 10.601
    add_frame(node, 10.20, 2000, 22)
    node.timer_callback(None)
    result = json.loads(node.report_candidate_pub.messages[1].data)
    assert result["stamp"] == 10.2 and result["cx"] == 8
    assert result["camera_point"]["z"] == pytest.approx(2)


def test_delayed_empty_event_clears_fused_target_without_using_live_flags(monkeypatch):
    node = make_node(monkeypatch)
    node.fusion_delay_s = .4
    send_candidate(node, 10, cx=None)
    send_candidate(node, 10.3, cx=8)
    node.target_detected = node.candidate_target_detected = True
    monkeypatch.setattr(rospy.Time, "now", staticmethod(lambda: stamp(10.401)))
    add_frame(node, 10, 1000, 11)
    node.timer_callback(None)
    assert not node.report_candidate_pub.messages
    assert not node.candidate_camera_point_pub.messages
    assert node.fusion_valid_pub.messages[-1].data is False


def test_delay_uses_newest_ready_event_after_timer_stall(monkeypatch):
    node = make_node(monkeypatch)
    node.fusion_delay_s = .4
    for s in [10, 10.05, 10.10, 10.15]:
        send_candidate(node, s)
    status, version = node.fusion_status_snapshot(stamp(10.511))
    assert status["stamp"] == 10.10 and version == 3
    assert len(node.pending_candidate_statuses) == 1
    assert node.fusion_status_snapshot(stamp(10.52)) == (status, version)


def test_zero_delay_keeps_immediate_fusion_and_old_frames_are_not_rejected(monkeypatch):
    node = make_node(monkeypatch)
    add_frame(node, 9.7, 1000, 11)
    send_candidate(node, 10)
    node.timer_callback(None)
    result = json.loads(node.report_candidate_pub.messages[0].data)
    assert result["stamp"] == 10
    assert result["source_timing"]["depth_dt_s"] == pytest.approx(-.3)
    assert not node.pending_candidate_statuses


def test_delayed_fusion_keeps_existing_nearest_frame_behavior_when_input_is_late(monkeypatch):
    node = make_node(monkeypatch)
    node.fusion_delay_s = .4
    add_frame(node, 9.7, 1000, 11)
    send_candidate(node, 10)
    monkeypatch.setattr(rospy.Time, "now", staticmethod(lambda: stamp(10.401)))
    node.timer_callback(None)
    result = json.loads(node.report_candidate_pub.messages[0].data)
    assert result["source_timing"]["depth_dt_s"] == pytest.approx(-.3)


def test_detector_records_time_before_detection_and_reuses_it(monkeypatch):
    node = detector.ThermalDetectorNode.__new__(detector.ThermalDetectorNode)
    for name, value in dict(publish_rate=0, process=None, frame_size=8, width=2, height=2,
                            shift_x=0, shift_y=0, roi_margin_x=0, roi_margin_y=0,
                            threshold_k=2, min_area=1, max_area=4, max_candidates=1,
                            min_local_contrast=0, display=False, reject_roi_border_touching=False,
                            stable_max_len=1).items():
        setattr(node, name, value)
    node.start = lambda: None
    node.stable_filter = detector.StableTargetFilter(enabled=False)
    node.multi_stable_filter = detector.MultiTargetFilter(enabled=False)
    for name in ("image_pub", "debug_pub", "pixel_pub", "detected_pub", "candidate_pixel_pub",
                 "candidate_detected_pub", "candidate_status_pub"):
        setattr(node, name, Publisher())
    clock = [10.0]
    monkeypatch.setattr(rospy.Time, "now", staticmethod(lambda: stamp(clock[0])))
    monkeypatch.setattr(rospy, "is_shutdown", Mock(side_effect=[False, True]))
    monkeypatch.setattr(detector, "read_frame", lambda *args: bytes(8))
    def detect(*args, **kwargs):
        clock[0] = 10.2  # Simulate 200 ms of processing.
        return {"detected": True, "cx": 1, "cy": 1, "bbox": [0, 0, 2, 2], "area": 4}
    monkeypatch.setattr(detector, "detect_hotspot", detect)
    monkeypatch.setattr(detector, "draw_debug", lambda *args, **kwargs: np.zeros((2, 2, 3), np.uint8))
    node.spin()
    for pub in (node.image_pub, node.debug_pub, node.pixel_pub, node.candidate_pixel_pub):
        assert pub.messages[0].header.stamp == stamp(10)
    assert json.loads(node.candidate_status_pub.messages[0].data)["stamp"] == 10
