import json
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
import rospy
from std_msgs.msg import String

from target_reporting.store import MissionStore
from test_exit_plane_filter import reporter, feed_target


def status(source, stamp=10, **extra):
    return {
        "detected": True, "candidate": True, "stable": False,
        "confirmable": False, "evidence_credible": True,
        "localization_valid": False, "stamp": stamp,
        "data": "AR14", "color": "red", "bbox": [20, 20, 40, 40],
        "camera_point": None, **extra,
    }


def send_status(node, source, data):
    msg = String(data=json.dumps(data))
    if source == "thermal":
        node._thermal_fused_candidate_cb(msg)
    else:
        node._status_cb(source, msg)


def send_image(node, source, stamp=10):
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    msg = node.bridge.cv2_to_imgmsg(image, encoding="bgr8")
    msg.header.stamp = rospy.Time.from_sec(stamp)
    node._image_cb(source, msg)


@pytest.mark.parametrize("source,kind", [("qr", "qr_code"), ("color", "color_tag"), ("thermal", "thermal_source")])
@pytest.mark.parametrize("image_first", [False, True])
def test_unlocalized_images_save_without_pose_and_never_enter_confirmation(reporter, source, kind, image_first):
    reporter.save_unlocalized_evidence = True
    if image_first:
        send_image(reporter, source)
    send_status(reporter, source, status(source))
    if not image_first:
        assert reporter.store.unlocalized_events() == []
        send_image(reporter, source)

    events = reporter.store.unlocalized_events()
    assert len(events) == 1
    assert events[0]["position"] is None
    assert events[0]["localization_label"] == "未定位"
    assert events[0]["spatial_filters"] == "not_evaluable_without_valid_position"
    path = Path(reporter.store.image_dir) / kind / events[0]["image"]["id"]
    image = cv2.imread(str(path))
    assert image.shape[0] > 240
    assert image[:240].any()  # The selected 2-D target is boxed.
    assert reporter.store.all_events() == []
    assert reporter.store.pending_events() == []
    assert reporter.store.pending_images() == []
    reporter.observation_pub.publish.assert_not_called()
    assert reporter.seq == 0

    # Repeated frames do not fill the directory, and normal confirmation can
    # still happen after depth recovers.
    send_image(reporter, source, 11)
    send_status(reporter, source, status(source, stamp=11))
    assert len(reporter.store.unlocalized_events()) == 1
    feed_target(reporter, source, 0.5, seconds=12)
    assert len(reporter.store.all_events()) == 1
    assert reporter.store.all_events()[0]["position"]["x"] == 0.5
    reopened = MissionStore(reporter.store.root, reporter.drone_id, reporter.mission_id)
    assert len(reopened.unlocalized_events()) == 1
    assert len(reopened.pending_events()) == 1
    assert all("unlocalized" not in name for name, _ in reopened.pending_images())


@pytest.mark.parametrize("override", [{"evidence_credible": False}, {"held": True}, {"detected": False}])
def test_untrusted_or_held_status_does_not_save(reporter, override):
    reporter.save_unlocalized_evidence = True
    send_image(reporter, "qr")
    send_status(reporter, "qr", status("qr", **override))
    assert reporter.store.unlocalized_events() == []


def test_disabled_and_mismatched_frame_do_not_save(reporter):
    send_image(reporter, "qr", 9)
    send_status(reporter, "qr", status("qr"))
    assert reporter.store.unlocalized_events() == []
    reporter.save_unlocalized_evidence = True
    send_status(reporter, "qr", status("qr"))
    assert reporter.store.unlocalized_events() == []
    send_status(reporter, "qr", status("qr", stamp=11))
    send_image(reporter, "qr", 10)  # Delayed frame must still match its own status.
    assert reporter.store.unlocalized_events()[0]["timestamp"] == 10


@pytest.mark.parametrize("source", ["qr", "color", "thermal"])
def test_disabled_unlocalized_saving_requires_depth_before_any_file_is_saved(reporter, source):
    reporter.save_unlocalized_evidence = False
    send_image(reporter, source, 10)
    send_status(reporter, source, status(source, stamp=10))
    reporter._try_save_unlocalized(source)
    assert reporter.store.unlocalized_events() == []
    assert reporter.store.all_events() == []
    assert list(Path(reporter.store.image_dir).rglob("*.jpg")) == []
    assert not reporter.pending_unlocalized[source]
    # The same source can still save a confirmed, localized event afterwards.
    feed_target(reporter, source, 0.5, seconds=12)
    events = reporter.store.all_events()
    assert len(events) == 1
    assert events[0]["position"]["x"] == 0.5
    assert list(Path(reporter.store.image_dir).rglob("*.jpg"))


def test_unlocalized_thermal_still_uses_leader_projection(reporter):
    reporter.save_unlocalized_evidence = True
    reporter._is_leader_thermal_source = Mock(return_value=True)
    data = status("thermal", camera_ray={"frame_id": "camera_optical", "x": 0.2, "y": 0.1, "z": 1.0})
    send_image(reporter, "thermal")
    send_status(reporter, "thermal", data)
    assert reporter.store.unlocalized_events() == []
    args = reporter._is_leader_thermal_source.call_args.args
    assert args[0] is None
    assert args[2].point.z == 1.0


def test_interval_or_movement_saves_new_images_and_preserves_old_files(reporter):
    reporter.save_unlocalized_evidence = True
    reporter.unlocalized_save_interval = 5.0
    reporter.unlocalized_center_shift = 80.0
    for stamp, cx in [(10, 40), (11, 45), (12, 130), (16, 135), (17, 135)]:
        send_image(reporter, "color", stamp)
        send_status(reporter, "color", status("color", stamp, center_u=cx, center_v=40))
    events = reporter.store.unlocalized_events()
    assert [e["timestamp"] for e in events] == [10, 12, 17]
    assert len({e["image"]["id"] for e in events}) == 3
    assert len(list(Path(reporter.store.image_dir).rglob("*.jpg"))) == 3
    assert reporter.store.all_events() == []
    reporter.observation_pub.publish.assert_not_called()


def test_restart_restores_repeat_suppression_and_continues_numbering(reporter):
    reporter.save_unlocalized_evidence = True
    send_image(reporter, "qr", 10)
    send_status(reporter, "qr", status("qr", 10))
    restarted = type(reporter)()
    restarted.save_unlocalized_evidence = True
    for stamp in (11, 15):
        send_image(restarted, "qr", stamp)
        send_status(restarted, "qr", status("qr", stamp))
    events = restarted.store.unlocalized_events()
    assert [e["evidence_seq"] for e in events] == [1, 2]
    assert [e["timestamp"] for e in events] == [10, 15]
    assert len(list(Path(restarted.store.image_dir).rglob("*.jpg"))) == 2


def test_legacy_unlocalized_filename_is_not_overwritten(reporter):
    old = {"target_type": "qr_code", "position": None, "localization_status": "unlocalized",
           "timestamp": 1, "result": {"center_u": 40, "center_v": 40}}
    reporter.store.append_unlocalized(old)
    path = Path(reporter.store.save_image("uav1_unlocalized_qr_code.jpg", b"old", "qr_code"))
    restarted = type(reporter)()
    restarted.save_unlocalized_evidence = True
    send_image(restarted, "qr", 10)
    send_status(restarted, "qr", status("qr", 10))
    assert path.read_bytes() == b"old"
    assert restarted.store.unlocalized_events()[-1]["evidence_seq"] == 2


def test_missing_leader_information_is_marked_incomplete(reporter):
    reporter.save_unlocalized_evidence = True
    reporter.leader_thermal_filter_enabled = True
    reporter.leader_thermal_projection_filter_enabled = True
    send_image(reporter, "thermal")
    send_status(reporter, "thermal", status("thermal"))
    assert reporter.store.unlocalized_events()[0]["leader_projection_status"] == "incomplete"
