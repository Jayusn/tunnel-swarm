#!/usr/bin/env python3
import importlib.util
import threading
from collections import defaultdict, deque
from pathlib import Path

import rospy
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
    reporter.thermal_ground_filter_enabled = True
    reporter.thermal_min_height = -0.08
    reporter.leader_odom_max_age = 10.0
    reporter.leader_odom_frame = "world"
    reporter.latest_leader_odom_received = rospy.Time.now()
    reporter.latest_leader_odom = Odometry()
    reporter.latest_leader_odom.header.frame_id = "world"
    reporter.latest_leader_odom.pose.pose.position.x = 1.0
    reporter.latest_leader_odom.pose.pose.position.y = 2.0
    return reporter


def test_filters_thermal_candidate_inside_leader_radius(monkeypatch):
    reporter = make_reporter(monkeypatch)
    assert reporter._is_leader_thermal_source({"x": 1.2, "y": 2.2, "z": 0.6})


def test_keeps_thermal_candidate_outside_leader_radius(monkeypatch):
    reporter = make_reporter(monkeypatch)
    assert not reporter._is_leader_thermal_source({"x": 1.4, "y": 2.0, "z": 0.6})


def test_keeps_candidate_when_leader_odometry_is_unavailable(monkeypatch):
    reporter = make_reporter(monkeypatch)
    reporter.latest_leader_odom = None
    assert not reporter._is_leader_thermal_source({"x": 1.0, "y": 2.0, "z": 0.6})


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
