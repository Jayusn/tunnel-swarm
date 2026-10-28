import copy
import importlib.util
import math
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from tf2_geometry_msgs import do_transform_point


SCRIPT = Path(__file__).parents[1] / "scripts/target_reporter_node.py"
SPEC = importlib.util.spec_from_file_location("exit_reporting_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def reporter(monkeypatch, tmp_path):
    params = {
        "~enable_remote_upload": False,
        "~exit_plane_filter_enabled": True,
        "~record_root": str(tmp_path),
        "~mission_id": "exit_filter_test",
        "~localization_frame": "world",
        "~thermal_confirm_hits": 1,
        "~thermal_confirmation_duration_s": 0.0,
    }
    monkeypatch.setattr(MODULE.rospy, "get_param", lambda key, default=None: params.get(key, default))
    for name in ("Publisher", "Subscriber", "Timer", "on_shutdown", "loginfo",
                 "loginfo_throttle", "logwarn_throttle"):
        monkeypatch.setattr(MODULE.rospy, name, MagicMock())
    monkeypatch.setattr(MODULE.rospy, "is_shutdown", lambda: False)
    buffer = MagicMock()

    def identity(point, frame, _timeout):
        result = copy.deepcopy(point)
        result.header.frame_id = frame
        return result

    buffer.transform.side_effect = identity
    monkeypatch.setattr(MODULE.tf2_ros, "Buffer", lambda **kwargs: buffer)
    monkeypatch.setattr(MODULE.tf2_ros, "TransformListener", MagicMock())
    return MODULE.TargetReporterNode()


def exit_pose(x=1.0, y=0.0, yaw=0.0, frame="world"):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = rospy.Time.from_sec(5)
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def point(x, y=0.0, z=1.0, frame="world"):
    msg = PointStamped()
    msg.header.frame_id = frame
    msg.header.stamp = rospy.Time.from_sec(10)
    msg.point.x, msg.point.y, msg.point.z = x, y, z
    return msg


@pytest.mark.parametrize("yaw", [0, math.pi / 2, math.pi, -math.pi / 2, .7])
def test_outward_side_and_boundary_use_exit_yaw_not_world_x(reporter, yaw):
    reporter._final_exit_pose_cb(exit_pose(2, -3, yaw))
    nx, ny = math.cos(yaw), math.sin(yaw)
    assert reporter._is_beyond_exit_plane(point(2 + nx, -3 + ny, 100))
    assert not reporter._is_beyond_exit_plane(point(2 - nx, -3 - ny, -100))
    assert not reporter._is_beyond_exit_plane(point(2, -3, 100))


def test_no_exit_or_disabled_keeps_targets(reporter):
    assert not reporter._is_beyond_exit_plane(point(100))
    reporter._final_exit_pose_cb(exit_pose())
    reporter.exit_plane_filter_enabled = False
    assert not reporter._is_beyond_exit_plane(point(100))


def test_coordinate_transform_includes_translation_rotation_and_event_time(reporter):
    reporter._final_exit_pose_cb(exit_pose(1, 1, math.pi / 2))
    transform = TransformStamped()
    transform.header.frame_id = "world"
    transform.child_frame_id = "UAV1/camera_init"
    transform.transform.translation.x = -1.2
    transform.transform.rotation.z = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    calls = []

    def tf(p, frame, _timeout):
        calls.append((p.header.frame_id, frame, p.header.stamp.to_sec()))
        return do_transform_point(p, transform)

    reporter.tf_buffer.transform.side_effect = tf
    assert reporter._is_beyond_exit_plane(point(2, frame="UAV1/camera_init"))
    assert not reporter._is_beyond_exit_plane(point(.5, frame="UAV1/camera_init"))
    assert calls == [("UAV1/camera_init", "world", 10.0)] * 2


def test_missing_tf_does_not_silently_treat_different_frames_as_same(reporter):
    reporter._final_exit_pose_cb(exit_pose())
    reporter.tf_buffer.transform.side_effect = RuntimeError("missing TF")
    assert not reporter._is_beyond_exit_plane(point(100, frame="camera_optical"))
    MODULE.rospy.logwarn_throttle.assert_called_once()


def test_invalid_exit_does_not_replace_known_plane(reporter):
    reporter._final_exit_pose_cb(exit_pose())
    known = reporter.exit_plane
    bad = exit_pose(100)
    bad.pose.orientation.w = 0
    reporter._final_exit_pose_cb(bad)
    assert reporter.exit_plane == known
    bad = exit_pose(100, frame="")
    reporter._final_exit_pose_cb(bad)
    assert reporter.exit_plane == known


def feed_target(reporter, source, x, seconds=10):
    p = point(x)
    p.header.stamp = rospy.Time.from_sec(seconds)
    reporter.latest_point[source] = p
    reporter.point_history[source].append(p)
    reporter.point_version[source] = reporter.point_version.get(source, 0) + 1
    reporter.status_version[source] = reporter.status_version.get(source, 0) + 1
    reporter.latest_status[source] = (
        {"_detector_confirmable": True, "_detector_stable": True}, 1.0
    )
    frame = np.zeros((100, 200, 3), np.uint8)
    reporter.images.add(source, seconds, frame)
    reporter.images.add("thermal_d435", seconds, frame)
    reporter._try_process(source)


@pytest.mark.parametrize("source", ["color", "qr", "thermal"])
def test_outside_targets_do_not_confirm_publish_or_save_any_evidence(reporter, source):
    reporter._final_exit_pose_cb(exit_pose())
    reporter.tracker.update = MagicMock(wraps=reporter.tracker.update)
    feed_target(reporter, source, 2)
    reporter.tracker.update.assert_not_called()
    reporter.observation_pub.publish.assert_not_called()
    assert reporter.seq == 0
    assert reporter.store.all_events() == []
    assert list(Path(reporter.store.image_dir).rglob("*.jpg")) == []
    # The raw detector/fusion evidence still exists in memory for display.
    assert reporter.images.has_match(source, 10, .03)


@pytest.mark.parametrize("source", ["color", "qr", "thermal"])
@pytest.mark.parametrize("mode", ["inside", "boundary", "no_exit", "disabled"])
def test_allowed_targets_still_save_normally(reporter, source, mode):
    x = 2
    if mode != "no_exit":
        reporter._final_exit_pose_cb(exit_pose())
    if mode == "disabled":
        reporter.exit_plane_filter_enabled = False
    if mode == "inside":
        x = 0
    if mode == "boundary":
        x = 1
    feed_target(reporter, source, x)
    assert len(reporter.store.all_events()) == 1
    assert len(list(Path(reporter.store.image_dir).rglob("*.jpg"))) == (2 if source == "thermal" else 1)
    reporter.observation_pub.publish.assert_called_once()


def test_receiving_exit_stops_future_records_without_deleting_old_ones(reporter):
    feed_target(reporter, "thermal", 2)
    old_files = list(Path(reporter.store.image_dir).rglob("*.jpg"))
    reporter._final_exit_pose_cb(exit_pose())
    feed_target(reporter, "thermal", 3, seconds=11)
    assert len(reporter.store.all_events()) == 1
    assert len(old_files) == 2 and all(p.exists() for p in old_files)
    assert len(list(Path(reporter.store.image_dir).rglob("*.jpg"))) == 2
