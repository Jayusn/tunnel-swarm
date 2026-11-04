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
        # 解析实际入口，但仅启动测试所需 NMPC 和路由器；不启动记录器或真实硬件。
        launch = Path(__file__).resolve().parents[1]/'launch/landing_nmpc_test.launch'
        config = roslaunch.config.load_config_default([str(launch)], None)
        for name, value in config.params.items():
            if name.startswith('/mpc_controller_node/'):
                rospy.set_param(self.prefix+'/controller/'+name.split('/mpc_controller_node/')[1], value.value)
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
        stage = 'FIXED_XY_DESCENT'
        drive(.3)
        ref = self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertAlmostEqual(ref.position.x, 1.1, places=4)
        self.assertAlmostEqual(ref.position.y, 2.2, places=4)
        self.assertAlmostEqual(ref.position.z, 6.8, places=4)
        # CH10 的近地判断也使用 FAST-LIO 相对起飞高度，飞控高度仍为 121 m。
        odom.pose.pose.position.z = 5.2
        rc.channels[9] = 1933
        drive(.6)
        self.assertIn('AUTO.LAND', mode_calls)


class FastlioPrecisionTest(LandingMemoryTest):
    use_fastlio = True


if __name__ == '__main__':
    # 每个测试基类管理自己的 master/rospy 生命周期，分别运行。
    if len(sys.argv) > 1 and sys.argv[1] == 'precision':
        suite = unittest.TestSuite([FastlioPrecisionTest('test_stopped_camera_preserves_compensated_world_goal')])
    else:
        suite = unittest.TestSuite([FastlioControllerTest('test_target_frame_and_takeoff_ramp')])
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
