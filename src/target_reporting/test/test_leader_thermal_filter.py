#!/usr/bin/env python3
import importlib.util
import threading
from pathlib import Path

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


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
    reporter.leader_thermal_exclusion_radius = 0.60
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
    assert reporter._is_leader_thermal_source({"x": 1.3, "y": 2.4, "z": 0.6})


def test_keeps_thermal_candidate_outside_leader_radius(monkeypatch):
    reporter = make_reporter(monkeypatch)
    assert not reporter._is_leader_thermal_source({"x": 1.7, "y": 2.0, "z": 0.6})


def test_keeps_candidate_when_leader_odometry_is_unavailable(monkeypatch):
    reporter = make_reporter(monkeypatch)
    reporter.latest_leader_odom = None
    assert not reporter._is_leader_thermal_source({"x": 1.0, "y": 2.0, "z": 0.6})
