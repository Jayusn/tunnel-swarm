#!/usr/bin/env python3
"""隔离 ROS master，模拟手动解锁和 OFFBOARD，验证独立测试起飞入口。"""
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


class OffboardStartTest(RcLandPriorityTest):
    def setUp(self):
        super().setUp()
        self.state = State(connected=True, armed=False, mode='POSCTL')
        self.pose = PoseStamped()
        self.pose.pose.orientation.w = 1.0
        self.pose.pose.position.x = 4.0
        self.pose.pose.position.y = 5.0
        self.pose.pose.position.z = 1.0
        self.landing_stage = 'IDLE'
        self.arm_requests, self.mode_requests = [], []
        self.state_pub = self.publisher('state', State)
        self.pose_pub = self.publisher('pose', PoseStamped)
        self.stage_pub = self.publisher('landing_state', String)
        for topic, kind in [('mission/status', String), ('trigger', Bool),
                            ('target', PoseStamped), ('prestream', PoseStamped)]:
            self.subscribe(topic, kind)
        if self._testMethodName != 'test_manual_start_needs_no_arming_or_mode_service':
            arm = rospy.Service(self.prefix+'/arm', CommandBool, self.arm)
            mode = rospy.Service(self.prefix+'/mode', SetMode, self.mode)
            self.addCleanup(arm.shutdown)
            self.addCleanup(mode.shutdown)
        ns = self.prefix+'/mission/'
        for key, value in {'allow_arming': False, 'auto_start': False,
                           'start_on_offboard': True, 'takeoff_height_m': 2.0,
                           'prestream_sec': 1.0, 'operation_timeout_sec': 2.0,
                           'hover_stable_sec': .1}.items():
            rospy.set_param(ns+key, value)
        self.launch('uav1_inplace_landing_test', 'uav1_test_landing_test_mission_node',
                    'mission', mavros_state_topic='state', local_pose_topic='pose',
                    position_setpoint_topic='target', offboard_prestream_topic='prestream',
                    landing_trigger_topic='trigger', landing_state_topic='landing_state',
                    arming_service='arm', set_mode_service='mode')
        self.timer = rospy.Timer(rospy.Duration(.02), self.publish)
        self.addCleanup(self.timer.shutdown)
        rospy.wait_for_service(ns+'start', timeout=5)
        self.wait(lambda: self.messages['prestream'] and self.messages['mission/status'],
                  'waiting position prestream')

    def publish(self, _):
        self.state.header.stamp = self.pose.header.stamp = rospy.Time.now()
        self.state_pub.publish(self.state)
        self.pose_pub.publish(self.pose)
        self.stage_pub.publish(String(data=self.landing_stage))

    def arm(self, request):
        self.arm_requests.append(request.value)
        return CommandBoolResponse(success=False, result=2)

    def mode(self, request):
        self.mode_requests.append(request.custom_mode)
        return SetModeResponse(mode_sent=False)

    def phase(self):
        return self.messages['mission/status'][-1].data

    def assert_no_automatic_flight_requests(self):
        self.assertEqual(self.arm_requests, [])
        self.assertEqual(self.mode_requests, [])

    def test_waits_for_manual_arm_and_offboard(self):
        # 等待超过起飞时限，仍只预流当前位置，不启动起飞计时。
        time.sleep(2.2)
        response = rospy.ServiceProxy(self.prefix+'/mission/start', Trigger)()
        self.assertFalse(response.success)
        self.state.mode = 'OFFBOARD'  # 只有 OFFBOARD、未手动解锁。
        time.sleep(.25)
        self.assertEqual(self.phase(), 'WAITING')
        self.state.mode = 'POSCTL'
        self.state.armed = True  # 只有解锁、未进入 OFFBOARD。
        time.sleep(.25)
        self.assertEqual(self.phase(), 'WAITING')
        self.assertEqual(self.messages['target'], [])
        self.assertFalse(any(m.data for m in self.messages['trigger']))
        self.assertGreater(len(self.messages['prestream']), 20)
        for msg in self.messages['prestream']:
            self.assertEqual(msg.pose.position, self.pose.pose.position)
        self.assert_no_automatic_flight_requests()

    def test_manual_offboard_takeoff_and_landing_handoff(self):
        self.state.armed = True
        self.state.mode = 'OFFBOARD'
        self.wait(lambda: self.phase() == 'TAKEOFF' and self.messages['target'],
                  'manual OFFBOARD takeoff')
        target = self.messages['target'][-1]
        self.assertEqual(target.pose.position.x, 4.0)
        self.assertEqual(target.pose.position.y, 5.0)
        self.assertEqual(target.pose.position.z, 3.0)  # 相对起飞点 +2 m。
        prestream_count = len(self.messages['prestream'])
        time.sleep(.2)
        self.assertEqual(len(self.messages['prestream']), prestream_count)
        self.pose.pose.position.z = 3.0
        self.wait(lambda: any(m.data for m in self.messages['trigger']), 'landing trigger')
        self.landing_stage = 'ACQUIRE'
        self.wait(lambda: self.phase() == 'COMPLETE', 'precision landing handoff')
        self.assertNotIn('PRESTREAM', [m.data for m in self.messages['mission/status']])
        self.assertNotIn('REQUEST_ARM', [m.data for m in self.messages['mission/status']])
        self.assert_no_automatic_flight_requests()

    def test_offboard_exit_keeps_existing_abort_behavior(self):
        self.state.armed = True
        self.state.mode = 'OFFBOARD'
        self.wait(lambda: self.phase() == 'TAKEOFF', 'takeoff')
        self.state.mode = 'POSCTL'
        self.wait(lambda: self.phase() == 'FAILSAFE_HOLD', 'existing mode exit handling')
        self.state.mode = 'OFFBOARD'
        time.sleep(.2)
        self.assertEqual(self.phase(), 'FAILSAFE_HOLD')
        self.assert_no_automatic_flight_requests()

    def test_manual_start_needs_no_arming_or_mode_service(self):
        self.state.armed = True
        self.state.mode = 'OFFBOARD'
        self.wait(lambda: self.phase() == 'TAKEOFF' and self.messages['target'],
                  'manual takeoff without flight command services')


if __name__ == '__main__':
    names = [name for name in OffboardStartTest.__dict__ if name.startswith('test_')]
    suite = unittest.TestSuite(OffboardStartTest(name) for name in names)
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
