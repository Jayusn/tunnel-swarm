#!/usr/bin/env python3
"""隔离 master，验证不同飞控/里程计坐标下的 FAST-LIO 起飞与精降目标。"""
import math
import subprocess
import sys
import time
import unittest
from pathlib import Path

import roslaunch
import roslib.packages
import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import AttitudeTarget, ExtendedState, PositionTarget, RCIn, State
from mavros_msgs.srv import SetMode, SetModeResponse
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from test_rc_land_priority_ros import RcLandPriorityTest
from test_landing_memory_ros import LandingMemoryTest


class FastlioControllerTest(RcLandPriorityTest):
    def test_target_frame_and_takeoff_ramp(self):
        self.exercise_controller(False)

    def test_synced_18cm_handoff_and_ch10(self):
        self.exercise_controller(True)

    def exercise_controller(self, synced):
        # 解析实际入口，但仅启动测试所需 NMPC 和路由器；不启动记录器或真实硬件。
        launch = Path(__file__).resolve().parents[1]/'launch/landing_nmpc_test.launch'
        if synced:
            launch = launch.with_name('landing_test.launch')
            config = roslaunch.config.load_config_default([(str(launch), ['marker_size_m:=0.18'])], None)
            self.assertEqual(config.params['/landing_test/takeoff_height_m'].value, 1.5)
            self.assertEqual(config.params['/mpc_controller_node/precision_landing/fixed_xy_Q_pos_xy'].value, 400.)
        else:
            config = roslaunch.config.load_config_default([str(launch)], None)
        for name, value in config.params.items():
            if name.startswith('/mpc_controller_node/'):
                rospy.set_param(self.prefix+'/controller/'+name.split('/mpc_controller_node/')[1], value.value)
            elif name.startswith('/landing_nmpc_test_router/'):
                rospy.set_param(self.prefix+'/router/'+name.split('/landing_nmpc_test_router/')[1], value.value)
        rospy.set_param(self.prefix+'/router/odometry_topic', self.prefix+'/odom')
        mode_calls = []
        service = rospy.Service(self.prefix+'/mode', SetMode,
            lambda req: (mode_calls.append(req.custom_mode) or SetModeResponse(mode_sent=True)))
        self.addCleanup(service.shutdown)
        mapping = {'~odom':'odom', '~drone_imu/data':'imu', '~mavros_local_pose':'local_pose',
                   '/mavros/set_mode':'mode', '/mavros/extended_state':'extended_state',
                   '/mavros/state':'mavros_state', '/mavros/rc/in':'rc',
                   '/mavros/setpoint_raw/attitude':'nmpc_attitude',
                   '~recovery_position_setpoint':'nmpc_recovery',
                   '~precision_landing_test_target':'nmpc_target',
                   '~rc_land_active':'rc_land', '~landing_search_state':'landing_state'}
        node = roslib.packages.find_node('uav1_inplace_landing_test', 'uav1_test_mpc_controller_node')[0]
        args = [node, '__name:=controller', '__ns:='+self.prefix]
        args += [k+':='+self.prefix+'/'+v for k,v in mapping.items()]
        self.processes.append(subprocess.Popen(args, stdout=self.log, stderr=self.log, start_new_session=True))
        self.launch('uav1_inplace_landing_test', 'landing_nmpc_test_router.py', 'router')
        for topic, kind in [('controller/mpc/reference_trajectory', RosPath),
                            ('mavros_position', PositionTarget), ('mavros_attitude', AttitudeTarget)]:
            self.subscribe(topic, kind)
        pubs = {topic:self.publisher(topic, kind) for topic,kind in [
            ('odom',Odometry),('imu',Imu),('local_pose',PoseStamped),('rc',RCIn),
            ('mavros_state',State),('extended_state',ExtendedState),('takeoff_position',PoseStamped),
            ('landing_position',PositionTarget),('landing_state',String)]}
        self.wait(lambda: pubs['odom'].get_num_connections() >= 2 and
                  pubs['rc'].get_num_connections() and pubs['landing_position'].get_num_connections(),
                  'controller/router subscriptions')
        odom = Odometry()
        odom.pose.pose.position = Point(1., 2., 5.)
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(.2), math.cos(.2)
        local = PoseStamped()
        local.pose.position = Point(100., 200., 21.)
        local.pose.orientation.w = 1.
        takeoff = PoseStamped()
        takeoff.pose.position = Point(1., 2., 7.)
        takeoff.pose.orientation.w = 1.
        target = PositionTarget(coordinate_frame=1, type_mask=2552, yaw=.4)
        target.position = Point(1.1, 2.2, 6.8)
        imu = Imu()
        imu.orientation = odom.pose.pose.orientation
        imu.linear_acceleration.z = 9.795
        state = State(connected=True, armed=False, mode='POSCTL')
        rc = RCIn(channels=[1500]*10)
        rc.channels[7] = rc.channels[9] = 1065
        extended = ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
        stage = 'IDLE'

        def drive(seconds):
            until = time.monotonic()+seconds
            while time.monotonic() < until:
                for topic,msg in [('odom',odom),('local_pose',local),('imu',imu),('rc',rc),
                                  ('mavros_state',state),('extended_state',extended)]:
                    msg.header.stamp = rospy.Time.now()
                    pubs[topic].publish(msg)
                pubs['landing_state'].publish(String(data=stage))
                topic,msg = ('takeoff_position',takeoff) if stage == 'IDLE' else ('landing_position',target)
                msg.header.stamp = rospy.Time.now()
                pubs[topic].publish(msg)
                time.sleep(.01)

        drive(.5)
        self.assertIsNone(self.last('mavros_attitude'))
        self.assertAlmostEqual(self.last('mavros_position').position.z, 21.)
        state.armed, state.mode = True, 'OFFBOARD'
        extended.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        drive(1.)
        self.assertIsNotNone(self.last('mavros_attitude'))
        ref = self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertGreater(ref.position.z, 5.25)
        self.assertLess(ref.position.z, 5.55)
        self.assertAlmostEqual(ref.position.x, 1., places=4)
        self.assertAlmostEqual(2*math.atan2(ref.orientation.z,ref.orientation.w), .4, places=4)
        local.pose.position.z = 121.  # 飞控高度漂移不改变 FAST-LIO 的起飞终点。
        drive(4.3)
        self.assertAlmostEqual(self.last('controller/mpc/reference_trajectory').poses[0].pose.position.z, 7., places=4)
        if synced:
            stage = 'ALIGN'
            target.type_mask = 2531
            target.velocity.x = .08
            drive(.8)
            before = self.last('controller/mpc/reference_trajectory').poses[0].pose.position
            stage = 'FIXED_XY_DESCENT'
            target.type_mask = 2552
            drive(.15)
            during = self.last('controller/mpc/reference_trajectory').poses[0].pose.position
            self.assertLess(abs(during.y-before.y), .05, 'XY must not jump at handoff')
            self.assertGreater(abs(during.y-target.position.y), .10)
            self.assertAlmostEqual(during.z, target.position.z, places=4)
            drive(1.6)
        stage = 'FIXED_XY_DESCENT'
        drive(.3)
        ref = self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertAlmostEqual(ref.position.x, 1.1, places=4)
        self.assertAlmostEqual(ref.position.y, 2.2, places=4)
        self.assertAlmostEqual(ref.position.z, 6.8, places=4)
        # CH10 恢复原近地 AUTO.LAND 交接，判断仍使用 FAST-LIO。
        odom.pose.pose.position.z = 5.2
        rc.channels[9] = 1933
        drive(.6)
        self.assertIn('AUTO.LAND', mode_calls)
        # 飞控确认 AUTO.LAND 后，路由器停止NMPC输出，避免继续争用末端控制权。
        state.mode = 'AUTO.LAND'
        drive(.3)
        count = len(self.messages['mavros_attitude'])
        drive(.3)
        self.assertEqual(len(self.messages['mavros_attitude']), count)


class FastlioPrecisionTest(LandingMemoryTest):
    use_fastlio = True


class FastlioTakeoffReferenceTest(FastlioPrecisionTest):
    use_takeoff_reference = True

    def setUp(self):
        # 从实际18 cm入口取末端参数；基类故意把飞控Z设为30 m，起飞FAST-LIO Z设为1 m。
        launch = Path(__file__).resolve().parents[1]/'launch/landing_test.launch'
        config = roslaunch.config.load_config_default([(str(launch), ['marker_size_m:=0.18'])], None)
        root = '/UAV1/precision_landing_node/'
        self.assertTrue(config.params[root+'auto_land_at_cutoff'].value)
        self.assertFalse(config.params['/landing_nmpc_test_router/offboard_touchdown'].value)
        self.assertEqual(config.params[root+'stages/auto_land/height_m'].value, .70)
        for key in ('stages/fixed_xy_descent/cutoff_height_m', 'control/use_fastlio_z_for_height'):
            rospy.set_param('/'+self._testMethodName+'/node/'+key, config.params[root+key].value)
        super().setUp()

    def test_takeoff_reference_handoff_with_marker_offset(self):
        expected = self.acquire()
        # 模拟落地后定位原点仍在码面上方；旧码面门限此时不会触发。
        marker_ground = 3. - self.targets[-1].pose.position.z
        self.assertGreater(1.19 - marker_ground, .3)
        self.frames = False
        self.x, self.y = expected
        self.wait(lambda: self.stage() == 'FIXED_XY_DESCENT', 'locked XY descent')
        self.z = 1.21
        time.sleep(.4)
        self.assertFalse(self.requests, 'above 0.2 m relative to FAST-LIO takeoff Z')
        self.assertAlmostEqual(self.commands[-1].position.x, expected[0], delta=.025)
        self.assertAlmostEqual(self.commands[-1].position.y, expected[1], delta=.025)
        self.z = 1.19
        self.wait(lambda: self.requests, 'takeoff-relative handoff with camera stopped')
        self.assertEqual(self.requests[-1], 'AUTO.LAND')

    def test_takeoff_reference_handoff_without_marker(self):
        self.have_marker = False
        self.trigger = True
        self.wait(lambda: self.stage() == 'FIXED_XY_DESCENT', 'no-marker descent')
        self.z = 1.21
        time.sleep(.4)
        self.assertFalse(self.requests)
        self.z = 1.19
        self.wait(lambda: self.requests, 'takeoff reference without marker ground')
        self.assertEqual(self.requests[-1], 'AUTO.LAND')


class FastlioOffboardDescentTest(FastlioPrecisionTest):
    use_takeoff_reference = True
    offboard_touchdown = True
    descent_mps = 1.

    def test_offboard_descent_uses_takeoff_floor(self):
        expected = self.acquire()
        self.frames = False
        self.x, self.y = expected
        self.wait(lambda: self.stage() == 'FIXED_XY_DESCENT', 'locked XY descent')
        # 先让下降目标到达近地减速区，再模拟机体跟随；不能瞬间把机体跳到目标下方。
        self.wait(lambda: self.commands[-1].position.z <= 1.15, 'target reaches near-ground band')
        self.z = 1.29
        time.sleep(.4)
        self.assertFalse(self.requests, 'must not switch AUTO.LAND at 0.3 m')
        self.z = 1.02
        self.wait(lambda: self.commands[-1].position.z < 1.14, 'near-ground slow descent')
        before = self.commands[-1]
        time.sleep(.5)
        after = self.commands[-1]
        dt = (after.header.stamp-before.header.stamp).to_sec()
        self.assertAlmostEqual((before.position.z-after.position.z)/dt, .1, delta=.015)
        self.wait(lambda: self.commands[-1].position.z <= .701, 'takeoff-relative descent floor')
        self.assertAlmostEqual(self.commands[-1].position.z, .70, places=3)
        self.assertAlmostEqual(self.commands[-1].position.x, expected[0], delta=.025)
        self.assertAlmostEqual(self.commands[-1].position.y, expected[1], delta=.025)
        self.assertFalse(self.requests, 'remain OFFBOARD at ground height')


if __name__ == '__main__':
    # 每个测试基类管理自己的 master/rospy 生命周期，分别运行。
    if len(sys.argv) > 1 and sys.argv[1] == 'offboard_descent':
        suite = unittest.TestSuite([FastlioOffboardDescentTest('test_offboard_descent_uses_takeoff_floor')])
    elif len(sys.argv) > 1 and sys.argv[1] == 'takeoff_reference':
        suite = unittest.TestSuite([
            FastlioTakeoffReferenceTest('test_takeoff_reference_handoff_with_marker_offset'),
            FastlioTakeoffReferenceTest('test_takeoff_reference_handoff_without_marker')])
    elif len(sys.argv) > 1 and sys.argv[1] == 'precision':
        suite = unittest.TestSuite([FastlioPrecisionTest('test_stopped_camera_preserves_compensated_world_goal')])
    else:
        suite = unittest.TestSuite([FastlioControllerTest('test_target_frame_and_takeoff_ramp'),
                                   FastlioControllerTest('test_synced_18cm_handoff_and_ch10')])
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
