"""Deterministic timing and request correlation tests; no ROS master needed."""
import json
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import point_to_point_mission as module
from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import rospy


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.clock = patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.now))
        self.clock.start()
        self.addCleanup(self.clock.stop)
        m = self.m = module.Mission.__new__(module.Mission)
        m.frame = 'UAV0/camera_init'
        m.start, m.goal = (.5, .5, .5), (4.5, 4.5, .5)
        m.start_wait, m.goal_wait = 2., 3.
        m.xy_tolerance, m.z_tolerance, m.speed_tolerance = .1, .02, .1
        m.lock = threading.RLock()
        m.status, m.goal_pub, m.mode_client = Mock(), Mock(), Mock()
        m.goal_pub.get_num_connections.return_value = 1
        m.state = State(connected=True, armed=True, mode='OFFBOARD')
        m.odom = Odometry()
        m.odom.header.frame_id = m.frame
        m.odom.pose.pose.position.x, m.odom.pose.pose.position.y, m.odom.pose.pose.position.z = m.start
        m.require_mpc_rc = False
        m.map_ready, m.phase, m.stable_since = True, 'WAIT_START', None
        m.request_id, m.trajectory_end, m.extended = None, None, None
        self.advance(10.)

    def advance(self, now, fresh=True):
        self.now = now
        if fresh:
            self.m.state_received = self.m.odom.header.stamp = rospy.Time.now()
            self.m.extended_received = rospy.Time.now()

    def test_mpc_command_gate_and_ch10_cancel(self):
        from mavros_msgs.msg import RCIn
        from std_msgs.msg import Bool
        m = self.m
        m.require_mpc_rc, m.rc = True, None
        m.rc_received = rospy.Time()
        self.assertFalse(m.mpc_command_ready(rospy.Time.now()))
        m.on_rc(RCIn(channels=[1500]*8+[1000,1000]))
        self.assertFalse(m.mpc_command_ready(rospy.Time.now()))
        m.on_rc(RCIn(channels=[1500]*8+[2000,1000]))
        self.assertTrue(m.mpc_command_ready(rospy.Time.now()))
        self.advance(11.)
        self.assertFalse(m.mpc_command_ready(rospy.Time.now()))
        m.on_rc_land(Bool(data=True))
        self.assertEqual(m.phase, 'CANCELLED')
        m.on_rc_land(Bool(data=False))
        m.tick(None)
        m.goal_pub.publish.assert_not_called()

    def test_manual_start_and_continuous_two_seconds(self):
        m = self.m
        m.state.armed = False
        m.tick(None)
        self.advance(20.)
        m.tick(None)
        m.goal_pub.publish.assert_not_called()
        m.state.armed = True
        m.tick(None)
        self.advance(21.)
        m.odom.twist.twist.linear.x = .2
        m.tick(None)
        m.odom.twist.twist.linear.x = 0.
        self.advance(22.)
        m.tick(None)
        self.advance(23.9)
        m.tick(None)
        m.goal_pub.publish.assert_not_called()
        self.advance(24.)
        m.tick(None)
        self.assertEqual(m.phase, 'PLANNING')
        goal = m.goal_pub.publish.call_args[0][0]
        self.assertEqual((goal.pose.position.x, goal.pose.position.y, goal.pose.position.z), m.goal)
        self.assertEqual(m.request_id, str(goal.header.stamp.to_nsec()))

    def test_result_correlation_failure_and_manual_takeover(self):
        m = self.m
        m.phase, m.request_id = 'PLANNING', '123'
        m.on_result(String(data=json.dumps(dict(start_ns='other', order=0))))
        self.assertEqual(m.phase, 'PLANNING')
        m.on_result(String(data=json.dumps(dict(start_ns='123', order=0, error='blocked'))))
        self.assertEqual(m.phase, 'FAILED')
        m.phase = 'PLANNING'
        m.state.mode = 'POSCTL'
        m.tick(None)
        self.assertEqual(m.phase, 'CANCELLED')
        m.mode_client.assert_not_called()

    def test_time_end_is_not_arrival_and_hover_resets_on_stale_odom(self):
        m = self.m
        m.phase, m.trajectory_end = 'FLYING', rospy.Time.from_sec(11.)
        self.advance(15.)
        m.tick(None)
        self.assertIsNone(m.stable_since)
        p = m.odom.pose.pose.position
        p.x, p.y, p.z = m.goal
        m.tick(None)
        self.advance(17., fresh=False)
        m.state_received = rospy.Time.now()
        m.tick(None)
        self.assertIsNone(m.stable_since)
        self.advance(18.)
        m.tick(None)
        with patch.object(module.threading, 'Thread') as worker:
            self.advance(20.9)
            m.tick(None)
            worker.assert_not_called()
            self.advance(21.)
            m.tick(None)
            worker.assert_called_once()
        self.assertEqual(m.phase, 'LAND_REQUESTED')

    def test_landing_requires_observed_mode_ground_and_disarm(self):
        m = self.m
        m.phase = 'LAND_REQUESTED'
        m.tick(None)
        self.assertEqual(m.phase, 'LAND_REQUESTED')
        m.state.mode = 'AUTO.LAND'
        m.tick(None)
        self.assertEqual(m.phase, 'LANDING')
        m.extended = ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
        m.tick(None)
        self.assertEqual(m.phase, 'LANDING')
        m.state.armed = False
        m.tick(None)
        self.assertEqual(m.phase, 'DONE')

    def test_rejected_landing_service_does_not_report_success(self):
        m = self.m
        m.phase = 'LAND_REQUESTED'
        p = m.odom.pose.pose.position
        p.x, p.y, p.z = m.goal
        m.mode_client.return_value.mode_sent = False
        m.request_land()
        m.mode_client.assert_called_once_with(base_mode=0, custom_mode='AUTO.LAND')
        self.assertEqual(m.phase, 'FAILED')


if __name__ == '__main__':
    unittest.main()
