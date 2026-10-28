import json
from pathlib import Path

import cv2
import numpy as np
import rospy
from std_msgs.msg import String

from test_exit_plane_filter import reporter


def fused(stamp, base_stamp, cx, x):
    return String(data=json.dumps({
        "detected": True, "candidate": True, "stable": True, "confirmable": True,
        "stamp": stamp, "source_timing": {"thermal_stamp": base_stamp},
        "cx": cx, "cy": 60, "bbox": [cx - 10, 50, 20, 20],
        "camera_point": {"frame_id": "camera_optical", "x": x, "y": 0, "z": 1},
    }))


def test_two_candidates_share_raw_frame_but_keep_their_own_boxes_and_color_images(reporter):
    reporter.thermal_fused_topic = "/thermal/report_candidate"
    base = 1793178126.9210522
    second = base + 1e-6
    raw = np.full((120, 320), 40, dtype=np.uint8)
    msg = reporter.bridge.cv2_to_imgmsg(raw, "mono8")
    msg.header.stamp = rospy.Time.from_sec(base)
    reporter._image_cb("thermal", msg)
    for stamp, cx, x, brightness in [(base, 60, .5, 80), (second, 220, 2.5, 160)]:
        color = np.full((120, 320, 3), brightness, dtype=np.uint8)
        reporter.images.add("thermal_d435", stamp, color)
        reporter._thermal_fused_candidate_cb(fused(stamp, base, cx, x))
    events = reporter.store.all_events()
    assert len(events) == 2
    for event, cx, other, brightness in zip(events, [60, 220], [220, 60], [80, 160]):
        root = Path(reporter.store.image_dir) / "thermal_source"
        thermal = cv2.imread(str(root / event["image"]["id"]))
        color = cv2.imread(str(root / event["d435_image"]["id"]))
        assert thermal[50, cx, 2] > 150
        assert abs(int(thermal[50, other, 2]) - 40) < 10
        assert abs(float(color[:100].mean()) - brightness) < 3
        assert event["result"]["center_u"] == cx


def test_sibling_candidate_color_image_cannot_confirm_or_replace_requested_image(reporter):
    reporter.thermal_fused_topic = "/thermal/report_candidate"
    base = 1793178126.9210522
    stamp = base + 1e-6
    frame = np.zeros((120, 320, 3), dtype=np.uint8)
    reporter.images.add("thermal", base, frame)
    reporter.images.add("thermal_d435", base, frame)  # Wrong candidate in same frame.
    msg = fused(stamp, base, 220, .5)
    reporter._thermal_fused_candidate_cb(msg)
    assert reporter.store.all_events() == []
    reporter.images.add("thermal_d435", stamp, frame)
    reporter._thermal_fused_candidate_cb(msg)
    assert len(reporter.store.all_events()) == 1
