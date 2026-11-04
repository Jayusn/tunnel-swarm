#!/usr/bin/env python3
"""隔离 ROS master：保持 OFFBOARD，按 FAST-LIO 接地判据输出零推力并普通上锁。"""
import sys
import time
import unittest

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget, ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandBoolResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String
from test_rc_land_priority_ros import RcLandPriorityTest


class OffboardTouchdownTest(RcLandPriorityTest):
    def setUp(self):
        super().setUp()
        self.requests = []
        self.accept = False
        service = rospy.Service(self.prefix+'/arming', CommandBool, self.disarm)
        self.addCleanup(service.shutdown)
        rospy.set_param(self.prefix+'/router/odometry_topic', self.prefix+'/odom')
        rospy.set_param(self.prefix+'/router/offboard_touchdown', True)
        self.launch('uav1_inplace_landing_test', 'landing_nmpc_test_router.py', 'router')
        for topic, kind in [('mavros_attitude', AttitudeTarget), ('owner', String)]:
            self.subscribe(topic, kind)
        self.pubs = {topic:self.publisher(topic, kind, latch=topic=='takeoff_reference')
                     for topic,kind in [('odom',Odometry),('local_pose',PoseStamped),
                         ('mavros_state',State),('extended_state',ExtendedState),
                         ('landing_state',String),('landing_position',PositionTarget),
                         ('nmpc_attitude',AttitudeTarget),('takeoff_reference',Float64),('rc_land',Bool)]}
        self.wait(lambda: self.pubs['odom'].get_num_connections() and
                  self.pubs['takeoff_reference'].get_num_connections() and
                  self.pubs['nmpc_attitude'].get_num_connections(), 'router connections')
        self.pubs['takeoff_reference'].publish(Float64(data=10.))
        self.z, self.vz = 11., 0.
        self.stage, self.rc_land = 'FIXED_XY_DESCENT', False
        self.armed = True
        self.odom_enabled = True
        self.landed = ExtendedState.LANDED_STATE_IN_AIR
        self.drive(.3)
        self.assertGreater(self.last('mavros_attitude').thrust, 0.)

    def disarm(self, request):
        self.requests.append((time.monotonic(), request.value))
        return CommandBoolResponse(success=self.accept, result=0 if self.accept else 2)

    def drive(self, seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            stamp = rospy.Time.now()
            odom = Odometry()
            odom.pose.pose.orientation.w = 1.
            odom.pose.pose.position.z = self.z
            odom.twist.twist.linear.z = self.vz
            local = PoseStamped()
            local.pose.orientation.w = 1.
            local.pose.position.z = 100.  # 与 FAST-LIO 故意不同，飞控高度不得参与判定。
            state = State(connected=True, armed=self.armed, mode='OFFBOARD')
            extended = ExtendedState(landed_state=self.landed)
            target = PositionTarget(coordinate_frame=1, type_mask=2552)
            target.position.z = 9.97
            attitude = AttitudeTarget(type_mask=128, thrust=.35)
            attitude.body_rate.x = .2
            self.pubs['landing_state'].publish(String(data=self.stage))
            self.pubs['rc_land'].publish(Bool(data=self.rc_land))
            for topic,msg in [('mavros_state',state),('extended_state',extended),
                              ('local_pose',local),('odom',odom),
                              ('landing_position',target),('nmpc_attitude',attitude)]:
                if topic == 'odom' and not self.odom_enabled:
                    continue
                msg.header.stamp = stamp
                self.pubs[topic].publish(msg)
            time.sleep(.02)

    def test_height_speed_dwell_and_ordinary_disarm_retry(self):
        # 高于原交接高度、超过接地高度误差、快速下降/上升均不触发停桨。
        for z,vz in [(10.3,0.),(10.06,0.),(9.94,0.),(10.02,-.2),(10.02,.2)]:
            self.z, self.vz = z,vz
            self.drive(.65)
            self.assertFalse(self.requests)
        self.z, self.vz = 10.02,0.
        self.drive(.25)
        self.assertFalse(self.requests, 'not stable for 0.5 seconds')
        self.vz = -.2
        self.drive(.1)
        self.vz = 0.
        started = time.monotonic()
        self.drive(1.1)
        self.assertTrue(self.requests)
        self.assertGreaterEqual(self.requests[0][0]-started, .48)
        self.assertTrue(all(value is False for _,value in self.requests))
        self.assertEqual(self.last('owner').data, 'TOUCHDOWN_DISARMING')
        self.assertEqual(self.last('mavros_attitude').thrust, 0.)
        self.assertEqual(self.last('mavros_attitude').body_rate.x, 0.)
        count = len(self.requests)
        self.accept = True
        self.drive(.65)
        self.assertGreater(len(self.requests), count, 'retry ordinary rejected disarm')
        self.assertEqual(self.last('mavros_attitude').thrust, 0.)
        self.armed = False
        self.drive(.2)
        self.assertEqual(self.last('owner').data, 'LANDED_DISARMED')
        count = len(self.requests)
        self.drive(.6)
        self.assertEqual(len(self.requests), count)

    def test_stale_odometry_resets_contact_dwell(self):
        self.z = 10.01
        self.drive(.25)
        self.odom_enabled = False
        self.drive(.65)
        self.assertFalse(self.requests, 'stale pose cannot prove contact')
        self.odom_enabled = True
        self.drive(.25)
        self.assertFalse(self.requests, 'new samples restart the 0.5 second interval')
        self.drive(1.)
        self.assertTrue(self.requests)

    def test_ch10_contact_and_wrong_stage(self):
        self.stage = 'ALIGN'
        self.z = 10.01
        self.drive(.7)
        self.assertFalse(self.requests, 'alignment is not descent')
        # CH10 会打断视觉精降；即使视觉阶段退出，仍按同一接地判据处理。
        self.rc_land = True
        self.drive(1.1)
        self.assertTrue(self.requests)
        self.assertEqual(self.last('mavros_attitude').thrust, 0.)


if __name__ == '__main__':
    names = [name for name in OffboardTouchdownTest.__dict__ if name.startswith('test_')]
    suite = unittest.TestSuite(OffboardTouchdownTest(name) for name in names)
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
