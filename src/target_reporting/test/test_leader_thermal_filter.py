#!/usr/bin/env python3
import importlib.util
import threading
from collections import defaultdict, deque
from pathlib import Path

import rospy
import pytest
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


SCRIPT = Path(__file__).parents[1] / "scripts/target_reporter_node.py"
SPEC = importlib.util.spec_from_file_location("target_reporter_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class IdentityTfBuffer:
    def transform(self, point, _target_frame, _timeout):
        transformed = PointStamped()
        transformed.header = point.header
        transformed.point = point.point
        return transformed


def make_reporter(monkeypatch):
    monkeypatch.setattr(
        MODULE.rospy.Time, "now", staticmethod(lambda: rospy.Time.from_sec(10.0))
    )
    reporter = MODULE.TargetReporterNode.__new__(MODULE.TargetReporterNode)
    reporter.lock = threading.Lock()
    reporter.tf_buffer = IdentityTfBuffer()
    reporter.localization_frame = "world"
    reporter.leader_thermal_filter_enabled = True
    reporter.leader_thermal_exclusion_radius = 0.35
    reporter.leader_thermal_projection_filter_enabled = True
    reporter.leader_thermal_projection_radius = 0.35
    reporter.thermal_ground_filter_enabled = True
    reporter.thermal_min_height = -0.08
    reporter.leader_odom_max_age = 10.0
    reporter.leader_odom_frame = "world"
    reporter.source_match_tolerance = 0.03
    reporter.leader_odom_match_tolerance = 0.06
    reporter.latest_leader_odom_received = rospy.Time.now()
    odom = Odometry()
    odom.header.frame_id = "world"
    odom.header.stamp = rospy.Time.from_sec(10.0)
    odom.pose.pose.position.x = 1.0
    odom.pose.pose.position.y = 2.0
    reporter.leader_odom_history = deque([odom], maxlen=400)
    return reporter


def test_filters_thermal_candidate_inside_leader_radius(monkeypatch):
    reporter = make_reporter(monkeypatch)
    assert reporter._is_leader_thermal_source({"x": 1.2, "y": 2.2, "z": 0.6}, 10.0)

def test_unlocalized_ray_checks_projection_without_inventing_target_distance(monkeypatch):
    reporter = make_reporter(monkeypatch)
    point = PointStamped()
    point.header.frame_id = "camera_optical"
    # Leader at (1, 2, 2), and a direction pointing at it.
    reporter.leader_odom_history[0].pose.pose.position.z = 2.0
    point.point.x, point.point.y, point.point.z = 0.5, 1.0, 1.0
    assert reporter._is_leader_thermal_source(None, 10.0, point)
    point.point.x = -0.5
    assert not reporter._is_leader_thermal_source(None, 10.0, point)


def test_keeps_thermal_candidate_outside_leader_radius(monkeypatch):
    reporter = make_reporter(monkeypatch)
    assert not reporter._is_leader_thermal_source({"x": 1.4, "y": 2.0, "z": 0.6}, 10.0)


def test_keeps_candidate_when_leader_odometry_is_unavailable(monkeypatch):
    reporter = make_reporter(monkeypatch)
    reporter.leader_odom_history.clear()
    assert not reporter._is_leader_thermal_source({"x": 1.0, "y": 2.0, "z": 0.6}, 10.0)


def test_delayed_detection_uses_historical_position_and_tf_time(monkeypatch):
    reporter = make_reporter(monkeypatch)
    older = Odometry()
    older.header.frame_id = "world"
    older.header.stamp = rospy.Time.from_sec(9.902)
    older.pose.pose.position.x = 0.5
    older.pose.pose.position.y = 2.0
    # 延迟到达的检测帧必须使用历史位置，而不是最新的 x=1.0。
    reporter.leader_odom_history.appendleft(older)
    transformed_stamps = []
    original_transform = reporter.tf_buffer.transform

    def transform(point, frame, timeout):
        transformed_stamps.append(point.header.stamp)
        return original_transform(point, frame, timeout)

    reporter.tf_buffer.transform = transform
    assert reporter._is_leader_thermal_source({"x": 0.5, "y": 2.0, "z": 0.6}, 9.9)
    assert not reporter._is_leader_thermal_source({"x": 1.0, "y": 2.0, "z": 0.6}, 9.9)
    assert transformed_stamps == [older.header.stamp, older.header.stamp]


@pytest.mark.parametrize(
    "event_stamp, expected",
    [(9.954, True), (10.046, True), (9.941, True), (10.059, True),
     (9.939, False), (10.061, False)],
)
def test_leader_timestamp_must_match_detection(monkeypatch, event_stamp, expected):
    reporter = make_reporter(monkeypatch)
    assert reporter._is_leader_thermal_source(
        {"x": 1.0, "y": 2.0, "z": 0.6}, event_stamp
    ) is expected


def test_leader_tolerance_does_not_change_detector_pairing(monkeypatch):
    reporter = make_reporter(monkeypatch)
    assert reporter.source_match_tolerance == 0.03
    assert reporter._is_leader_thermal_source({"x": 1.0, "y": 2.0}, 10.046)
    reporter.leader_odom_match_tolerance = 0.03
    assert not reporter._is_leader_thermal_source({"x": 1.0, "y": 2.0}, 10.046)


def test_leader_callback_keeps_history_for_delayed_detection(monkeypatch):
    reporter = make_reporter(monkeypatch)
    newer = Odometry()
    newer.header.stamp = rospy.Time.from_sec(10.1)
    newer.header.frame_id = "world"
    newer.pose.pose.position.x = 5.0
    reporter._leader_odom_cb(newer)
    assert len(reporter.leader_odom_history) == 2
    assert reporter._is_leader_thermal_source({"x": 1.0, "y": 2.0, "z": 0.6}, 10.0)


def test_stale_leader_stream_still_skips_filter(monkeypatch):
    reporter = make_reporter(monkeypatch)
    reporter.leader_odom_max_age = 0.5
    reporter.latest_leader_odom_received = rospy.Time.from_sec(9.0)
    assert not reporter._is_leader_thermal_source(
        {"x": 1.0, "y": 2.0, "z": 0.6}, 10.0
    )


def test_filters_thermal_candidate_below_ground_height(monkeypatch):
    reporter = make_reporter(monkeypatch)
    assert reporter._is_ground_thermal_source({"x": 1.0, "y": 2.0, "z": -0.081})


def test_keeps_thermal_candidate_at_ground_height_limit(monkeypatch):
    reporter = make_reporter(monkeypatch)
    assert not reporter._is_ground_thermal_source(
        {"x": 1.0, "y": 2.0, "z": -0.08}
    )


def test_keeps_ground_candidate_when_filter_is_disabled(monkeypatch):
    reporter = make_reporter(monkeypatch)
    reporter.thermal_ground_filter_enabled = False
    assert not reporter._is_ground_thermal_source(
        {"x": 1.0, "y": 2.0, "z": -0.20}
    )


def test_fused_thermal_candidate_updates_point_and_status_atomically():
    reporter = MODULE.TargetReporterNode.__new__(MODULE.TargetReporterNode)
    reporter.lock = threading.Lock()
    reporter.latest_point = {}
    reporter.point_history = defaultdict(lambda: deque(maxlen=30))
    reporter.point_version = {}
    reporter.latest_status = {}
    reporter.status_version = {}
    processed = []
    reporter._try_process = processed.append

    reporter._thermal_fused_candidate_cb(
        String(
            data=(
                '{"detected":true,"candidate":true,"stable":true,'
                '"confirmable":true,"stamp":12.5,"cx":100,"cy":80,'
                '"bbox":[90,70,20,20],"camera_point":'
                '{"frame_id":"camera_optical","x":1.0,"y":2.0,"z":3.0}}'
            )
        )
    )

    assert processed == ["thermal"]
    assert reporter.latest_point["thermal"].header.stamp.to_sec() == 12.5
    assert reporter.latest_point["thermal"].point.z == 3.0
    assert reporter.latest_status["thermal"][0]["_detector_confirmable"] is True


def camera_point(x, y, z):
    point = PointStamped()
    point.header.frame_id = "camera_optical"
    point.header.stamp = rospy.Time.from_sec(10.0)
    point.point.x, point.point.y, point.point.z = x, y, z
    return point


def projection_reporter(monkeypatch, leader_xyz=(-0.272, -0.269, 1.740)):
    reporter = make_reporter(monkeypatch)
    stamps = []

    def transform(point, frame, timeout):
        stamps.append((frame, point.header.stamp.to_sec()))
        if frame == "camera_optical":
            return camera_point(*leader_xyz)
        return point

    reporter.tf_buffer.transform = transform
    return reporter, stamps


def test_projection_filters_wrong_depth_in_leader_direction(monkeypatch):
    reporter, stamps = projection_reporter(monkeypatch)
    point = camera_point(-0.067823, -0.083153, 0.578)
    # World distance is outside the original radius; the direction still
    # falls on UAV0 even though foreground fallback supplied the wrong depth.
    assert reporter._is_leader_thermal_source({"x": 4.0, "y": 2.0}, 10.046, point)
    assert stamps[-1][0] == "camera_optical"
    assert stamps[-1][1] == pytest.approx(10.046)


def test_projection_switch_off_retains_spatial_filter(monkeypatch):
    reporter, stamps = projection_reporter(monkeypatch)
    reporter.leader_thermal_projection_filter_enabled = False
    point = camera_point(-0.067823, -0.083153, 0.578)
    assert not reporter._is_leader_thermal_source({"x": 4.0, "y": 2.0}, 10.0, point)
    assert all(frame != "camera_optical" for frame, _ in stamps)
    assert reporter._is_leader_thermal_source({"x": 1.0, "y": 2.0}, 10.0, point)


@pytest.mark.parametrize("scale", [0.3, 1.0, 3.0])
def test_projection_ignores_depth_but_keeps_targets_in_other_directions(monkeypatch, scale):
    reporter, _ = projection_reporter(monkeypatch, (0.0, 0.0, 2.0))
    leader = camera_point(1.0, 2.0, 0.0)
    assert reporter._is_in_leader_projection(leader, camera_point(.1*scale, 0, scale), 10.0)
    assert not reporter._is_in_leader_projection(leader, camera_point(.3*scale, 0, scale), 10.0)


@pytest.mark.parametrize("leader_xyz,candidate_xyz", [
    ((0, 0, -1), (0, 0, 1)),
    ((0, 0, 1), (0, 0, 0)),
    ((0, 0, 1), (float('nan'), 0, 1)),
])
def test_projection_needs_forward_finite_camera_points(monkeypatch, leader_xyz, candidate_xyz):
    reporter, _ = projection_reporter(monkeypatch, leader_xyz)
    assert not reporter._is_leader_thermal_source(
        {"x": 4.0, "y": 2.0}, 10.0, camera_point(*candidate_xyz))


def test_projection_needs_matched_fresh_odometry(monkeypatch):
    reporter, stamps = projection_reporter(monkeypatch)
    point = camera_point(-.067823, -.083153, .578)
    assert not reporter._is_leader_thermal_source({"x": 4.0, "y": 2.0}, 10.1, point)
    assert not stamps
    reporter.latest_leader_odom_received = rospy.Time.from_sec(8)
    reporter.leader_odom_max_age = .5
    assert not reporter._is_leader_thermal_source({"x": 4.0, "y": 2.0}, 10.0, point)
    assert not stamps


def test_projection_tf_failure_does_not_drop_unrelated_target(monkeypatch):
    reporter, _ = projection_reporter(monkeypatch)
    original = reporter.tf_buffer.transform
    def transform(point, frame, timeout):
        if frame == 'camera_optical':
            raise RuntimeError('missing transform')
        return original(point, frame, timeout)
    reporter.tf_buffer.transform = transform
    assert not reporter._is_leader_thermal_source(
        {"x": 4.0, "y": 2.0}, 10.0, camera_point(0, 0, 1))
