#!/usr/bin/env python3
"""隔离 master 验证独立 NMPC 不会用电机启动瞬态学习推力映射。"""
import math
from pathlib import Path
import subprocess
import time
import unittest

import roslaunch
import roslib.packages
import rospy
from mavros_msgs.msg import AttitudeTarget, ESCStatus, ESCStatusItem, ExtendedState, PositionTarget, RCIn, State
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Imu
from test_rc_land_priority_ros import RcLandPriorityTest


class ThrustStartupTest(RcLandPriorityTest):
    def test_waits_for_actual_climb_and_settling(self):
        launch = Path(__file__).resolve().parents[1] / 'launch/landing_nmpc_test.launch'
        config = roslaunch.config.load_config_default([str(launch)], None)
        for key, param in config.params.items():
            if key.startswith('/mpc_controller_node/'):
                rospy.set_param(self.prefix + '/controller/' + key.split('/mpc_controller_node/')[1], param.value)
        # 单独检查推力学习：不让外力补偿改变模拟机体的输出。
        rospy.set_param(self.prefix + '/controller/force_estimator/enable_disturbance_compensation', False)
        mapping = {'~odom': 'odom', '~drone_imu/data': 'imu',
                   '/mavros/state': 'state', '/mavros/extended_state': 'extended',
                   '/mavros/rc/in': 'rc', '/mavros/esc_status': 'rpm',
                   '~precision_landing_test_target': 'target',
                   '/mavros/setpoint_raw/attitude': 'attitude'}
        executable = roslib.packages.find_node('uav1_inplace_landing_test', 'uav1_test_mpc_controller_node')[0]
        args = [executable, '__name:=controller', '__ns:=' + self.prefix]
        args += [key + ':=' + self.prefix + '/' + value for key, value in mapping.items()]
        logs = []
        self.handles.append(rospy.Subscriber('/rosout', Log, logs.append))
        self.subscribe('attitude', AttitudeTarget)
        self.processes.append(subprocess.Popen(args, stdout=self.log, stderr=self.log, start_new_session=True))
        pubs = {key: self.publisher(key, kind) for key, kind in [
            ('odom', Odometry), ('imu', Imu), ('state', State), ('extended', ExtendedState),
            ('rc', RCIn), ('rpm', ESCStatus), ('target', PositionTarget)]}
        self.wait(lambda: all(p.get_num_connections() for p in pubs.values()), 'controller inputs')
        odom = Odometry()
        odom.pose.pose.position.z = 5.0  # 非零原点，必须检查相对起飞高度。
        odom.pose.pose.orientation.w = 1.0
        imu = Imu()
        imu.orientation.w = 1.0
        imu.linear_acceleration.z = 9.795
        state = State(connected=True, armed=False, mode='POSCTL')
        extended = ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
        rc = RCIn(channels=[1500] * 10)
        rc.channels[7] = rc.channels[9] = 1065
        target = PositionTarget(coordinate_frame=1, type_mask=2552)
        rpm = ESCStatus(esc_status=[ESCStatusItem(rpm=11000) for _ in range(4)])

        def drive(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                target.position = odom.pose.pose.position
                for key, msg in [('odom', odom), ('imu', imu), ('state', state),
                                 ('extended', extended), ('rc', rc), ('rpm', rpm), ('target', target)]:
                    msg.header.stamp = rospy.Time.now()
                    pubs[key].publish(msg)
                time.sleep(.01)

        def learning_started():
            return any(m.name == self.prefix + '/controller' and '在线映射已更新' in m.msg for m in logs)

        drive(.3)
        state.armed, state.mode = True, 'OFFBOARD'
        extended.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        # 重现实飞问题：已报 IN_AIR，仍贴地且 RPM 未稳定，持续超过旧学习启动时间。
        drive(1.2)
        self.assertFalse(learning_started())
        self.assertIsNotNone(self.last('attitude'))
        self.assertLess(max(m.thrust for m in self.messages['attitude']), .57)
        odom.twist.twist.linear.z = .4
        for _ in range(20):
            odom.pose.pose.position.z += .02
            drive(.05)
        self.assertFalse(learning_started(), 'climb must not unlock learning')
        odom.twist.twist.linear.z = 0.0
        drive(.25)
        self.assertFalse(learning_started(), 'brief settling is not sufficient')
        odom.twist.twist.linear.z = .25
        drive(.1)
        odom.twist.twist.linear.z = 0.0
        drive(.3)
        self.assertFalse(learning_started(), 'interrupted settling must restart its timer')
        # 空中稳定转速对应 0.60 悬停比例，检查门控并非永久关闭学习。
        steady_rpm = math.sqrt(1.926 * 9.795 / .60 * .543 / (4 * 1.835e-8))
        for item in rpm.esc_status:
            item.rpm = int(steady_rpm)
        drive(.65)
        self.assertTrue(learning_started(), 'stable airborne samples must enable learning')


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.TestSuite([ThrustStartupTest('test_waits_for_actual_climb_and_settling')]))
    raise SystemExit(not result.wasSuccessful())
