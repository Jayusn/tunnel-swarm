#!/usr/bin/env python3
"""隔离 ROS master：区分慢降与接触，组合判据确认后平滑减推力并普通上锁。"""
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
        self.target_z, self.thrust = 9.8, .6
        self.stage, self.rc_land = 'ALIGN', False
        self.armed = True
        self.odom_enabled = True
        self.target_enabled = True
        self.attitude_enabled = True
        self.z_rate = 0.
        self.landed = ExtendedState.LANDED_STATE_IN_AIR
        self.drive(.5)
        self.assertGreater(self.last('mavros_attitude').thrust, 0.)
        self.stage, self.thrust = 'FIXED_XY_DESCENT', .35

    def disarm(self, request):
        self.requests.append((time.monotonic(), request.value))
        return CommandBoolResponse(success=self.accept, result=0 if self.accept else 2)

    def drive(self, seconds):
        until = time.monotonic()+seconds
        previous = time.monotonic()
        while time.monotonic() < until:
            now = time.monotonic()
            self.z += self.z_rate*(now-previous)
            previous = now
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
            target.position.z = self.target_z
            attitude = AttitudeTarget(type_mask=128, thrust=self.thrust)
            attitude.body_rate.x = .2
            self.pubs['landing_state'].publish(String(data=self.stage))
            self.pubs['rc_land'].publish(Bool(data=self.rc_land))
            for topic,msg in [('mavros_state',state),('extended_state',extended),
                              ('local_pose',local),('odom',odom),
                              ('landing_position',target),('nmpc_attitude',attitude)]:
                if topic == 'odom' and not self.odom_enabled:
                    continue
                if topic == 'landing_position' and not self.target_enabled:
                    continue
                if topic == 'nmpc_attitude' and not self.attitude_enabled:
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
        self.assertFalse(self.requests, 'not stable for 0.6 seconds')
        self.vz = -.2
        self.drive(.1)
        self.vz = 0.
        started = time.monotonic()
        mark = len(self.messages['mavros_attitude'])
        self.drive(1.8)
        self.assertTrue(self.requests)
        self.assertGreaterEqual(self.requests[0][0]-started, .9)
        outputs = self.messages['mavros_attitude'][mark:]
        self.assertTrue(any(0.01 < m.thrust < .34 for m in outputs), 'ramp before zero thrust')
        for before, after in zip(outputs, outputs[1:]):
            dt = (after.header.stamp-before.header.stamp).to_sec()
            self.assertLessEqual(before.thrust-after.thrust, .35/.3*max(dt,0.)+.025)
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
        self.assertFalse(self.requests, 'new samples restart the 0.6 second interval')
        self.drive(1.8)
        self.assertTrue(self.requests)

    def test_slow_descent_and_missing_unloading_do_not_disarm(self):
        # 实飞提前停推力片段：虽低于 5 cm，但仍慢降，推力接近悬停值。
        self.z, self.vz, self.z_rate, self.thrust = 10.045, -.026, -.026, .56
        self.drive(1.1)
        self.assertFalse(self.requests)
        # 即使推力已降低，真实高度持续变化也不能用小 vz 代替稳定高度窗口。
        self.z, self.vz, self.z_rate, self.thrust = 10.04, -.02, -.025, .35
        self.drive(1.1)
        self.assertFalse(self.requests)
        self.z, self.vz, self.z_rate, self.thrust = 10.01, 0., 0., .56
        self.drive(1.3)
        self.assertFalse(self.requests, 'stable low hover is not contact')
        self.thrust = .35
        self.target_z = 10.0
        self.drive(1.3)
        self.assertFalse(self.requests, 'no downward position error')
        self.target_z = 9.8
        self.drive(1.8)
        self.assertTrue(self.requests)

    def test_missing_hover_reference_does_not_disarm(self):
        # 新起飞基准清空上一次空中推力标定，缺失本次基准时不能凭低推力停桨。
        self.pubs['takeoff_reference'].publish(Float64(data=10.))
        self.z = 10.01
        self.drive(1.8)
        self.assertFalse(self.requests)

    def test_stale_target_does_not_prove_contact(self):
        self.target_enabled = False
        self.drive(.4)
        self.z = 10.01
        self.drive(1.3)
        self.assertFalse(self.requests)
        self.target_enabled = True
        self.drive(1.8)
        self.assertTrue(self.requests)

    def test_ch10_preempts_pending_contact_and_wrong_stage(self):
        self.stage = 'ALIGN'
        self.z = 10.01
        self.drive(.7)
        self.assertFalse(self.requests, 'alignment is not descent')
        self.stage = 'FIXED_XY_DESCENT'
        self.drive(.25)
        self.assertFalse(self.requests)
        # 接地计时尚未完成时 CH10 接管，即使视觉阶段还未更新也不能抢先上锁。
        self.rc_land = True
        self.drive(1.1)
        self.assertFalse(self.requests)
        self.assertGreater(self.last('mavros_attitude').thrust, 0.)

    def test_ch10_interrupts_thrust_ramp(self):
        self.z = 10.01
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            self.drive(.02)
            if self.last('owner').data == 'TOUCHDOWN_RAMP_DOWN':
                break
        self.assertEqual(self.last('owner').data, 'TOUCHDOWN_RAMP_DOWN')
        self.rc_land = True
        self.drive(.8)
        self.assertFalse(self.requests, 'CH10 must cancel pending direct disarm')
        self.assertAlmostEqual(self.last('mavros_attitude').thrust, self.thrust, places=4)


if __name__ == '__main__':
    names = [name for name in OffboardTouchdownTest.__dict__ if name.startswith('test_')]
    suite = unittest.TestSuite(OffboardTouchdownTest(name) for name in names)
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
