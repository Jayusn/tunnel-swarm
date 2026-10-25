#!/usr/bin/env python3
"""隔离 ROS master，用模拟飞控验证解锁等待与起飞交接分别计时。

source /home/oem/match_ws/devel/setup.bash
/usr/bin/python3 src/uav1_inplace_landing_test/test/test_takeoff_timeout_ros.py
"""
import sys
import time
import unittest

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from test_rc_land_priority_ros import RcLandPriorityTest


class TakeoffTimeoutTest(RcLandPriorityTest):
    def setUp(self):
        super().setUp()
        self.state = State(connected=True, armed=False, mode='STABILIZED')
        self.pose = PoseStamped()
        self.pose.pose.orientation.w = 1.0
        self.accept_arm = False
        self.arm_requests = 0
        self.landing_stage = 'IDLE'
        self.state_pub = self.publisher('state', State)
        self.pose_pub = self.publisher('pose', PoseStamped)
        self.stage_pub = self.publisher('landing_state', String)
        for topic, kind in [('mission/status', String), ('trigger', Bool),
                            ('target', PoseStamped)]:
            self.subscribe(topic, kind)
        arm = rospy.Service(self.prefix+'/arm', CommandBool, self.arm)
        mode = rospy.Service(self.prefix+'/mode', SetMode, self.mode)
        self.addCleanup(arm.shutdown)
        self.addCleanup(mode.shutdown)
        # 缩短测试时限，保留真实节点的预流、解锁、起飞、交接逻辑。
        ns = self.prefix+'/mission/'
        for key, value in {'allow_arming': True, 'auto_start': False,
                           'prestream_sec': 1.0, 'operation_timeout_sec': 6.0,
                           'takeoff_height_m': 1.7, 'hover_stable_sec': .1}.items():
            rospy.set_param(ns+key, value)
        self.launch('uav1_inplace_landing_test', 'uav1_test_landing_test_mission_node',
                    'mission', mavros_state_topic='state', local_pose_topic='pose',
                    position_setpoint_topic='target', landing_trigger_topic='trigger',
                    landing_state_topic='landing_state', arming_service='arm',
                    set_mode_service='mode')
        self.timer = rospy.Timer(rospy.Duration(.02), self.publish)
        self.addCleanup(self.timer.shutdown)
        rospy.wait_for_service(ns+'start', timeout=5)
        self.wait(lambda: self.state_pub.get_num_connections() and
                  self.pose_pub.get_num_connections() and
                  self.stage_pub.get_num_connections() and
                  self.messages['mission/status'], 'mission subscriptions')
        time.sleep(.15)
        self.started = time.monotonic()
        response = rospy.ServiceProxy(ns+'start', Trigger)()
        self.assertTrue(response.success, response.message)

    def publish(self, _):
        self.state.header.stamp = self.pose.header.stamp = rospy.Time.now()
        self.state_pub.publish(self.state)
        self.pose_pub.publish(self.pose)
        self.stage_pub.publish(String(data=self.landing_stage))

    def arm(self, request):
        self.arm_requests += 1
        if self.accept_arm and request.value:
            self.state.armed = True
        return CommandBoolResponse(success=self.state.armed, result=0 if self.state.armed else 2)

    def mode(self, request):
        self.state.mode = request.custom_mode
        return SetModeResponse(mode_sent=True)

    def phase(self):
        return self.messages['mission/status'][-1].data

    def test_delayed_arm_still_hands_off_after_original_deadline(self):
        time.sleep(max(0, self.started+4.0-time.monotonic()))
        self.assertEqual(self.phase(), 'REQUEST_ARM')
        self.assertGreater(self.arm_requests, 1)
        self.accept_arm = True
        self.wait(lambda: self.phase() == 'TAKEOFF', 'armed takeoff')
        time.sleep(2.4)
        self.assertGreater(time.monotonic()-self.started, 6.0)
        self.assertEqual(self.phase(), 'TAKEOFF')
        self.assertFalse(any(m.data for m in self.messages['trigger']))
        self.pose.pose.position.z = 1.7
        self.wait(lambda: any(m.data for m in self.messages['trigger']), 'landing trigger')
        self.landing_stage = 'ACQUIRE'
        self.wait(lambda: self.phase() == 'COMPLETE', 'landing handoff')
        self.assertNotIn('FAILSAFE_HOLD', [m.data for m in self.messages['mission/status']])

    def test_unarmed_wait_still_times_out(self):
        self.wait(lambda: self.phase() == 'FAILSAFE_HOLD', 'unarmed timeout')
        self.assertFalse(self.state.armed)
        self.assertFalse(any(m.data for m in self.messages['trigger']))

    def test_takeoff_still_times_out_when_never_at_target(self):
        self.accept_arm = True
        self.wait(lambda: self.phase() == 'TAKEOFF', 'armed takeoff')
        armed_at = time.monotonic()
        self.wait(lambda: self.phase() == 'FAILSAFE_HOLD', 'takeoff timeout')
        self.assertGreater(time.monotonic()-armed_at, 5.7)
        self.assertFalse(any(m.data for m in self.messages['trigger']))


if __name__ == '__main__':
    suite = unittest.TestSuite(TakeoffTimeoutTest(name) for name in [
        'test_delayed_arm_still_hands_off_after_original_deadline',
        'test_unarmed_wait_still_times_out',
        'test_takeoff_still_times_out_when_never_at_target'])
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
