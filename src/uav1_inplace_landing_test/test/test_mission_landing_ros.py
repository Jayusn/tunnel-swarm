#!/usr/bin/env python3
"""主任务控制器+仲裁器隔离测试；不启动实机、起飞测试或自动解锁。"""
import math
import subprocess
import sys
import time
import unittest
from pathlib import Path
import rospy
import roslib.packages
import rospkg
import yaml
from test_rc_land_priority_ros import RcLandPriorityTest
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu
from mavros_msgs.msg import AttitudeTarget, PositionTarget, RCIn, State, ExtendedState
from std_msgs.msg import Bool, String


class MissionLandingTest(RcLandPriorityTest):
    def test_precision_uses_fastlio_for_acquire_and_marker_loss(self):
        import cv2
        import numpy as np
        from cv_bridge import CvBridge
        from sensor_msgs.msg import CameraInfo, Image
        from test_precision_landing_node import generate_aruco_marker

        ns = self.prefix + '/precision'
        for key, value in {
            'topics/odometry': self.prefix + '/odom',
            'topics/local_pose': self.prefix + '/px4_pose',
            'topics/image': self.prefix + '/image',
            'topics/camera_info': self.prefix + '/camera_info',
            'topics/mavros_state': self.prefix + '/state',
            'topics/trigger': self.prefix + '/trigger',
            'topics/setpoint': self.prefix + '/setpoint',
            'topics/landing_state': self.prefix + '/stage',
            'topics/set_mode_service': self.prefix + '/set_mode',
            'marker/size_m': 0.5,
            'state_machine/acquire_stable_sec': 0.05,
            'state_machine/align_stable_sec': 0.05,
            'state_machine/high_height_m': 0.5,
            'state_machine/auto_land_height_m': 0.5,
        }.items():
            rospy.set_param(ns + '/' + key, value)
        self.launch('uav1_inplace_landing_test', 'uav1_test_precision_landing_node', 'precision')
        self.subscribe('setpoint', PositionTarget)
        self.subscribe('stage', String)
        pubs = {t: self.publisher(t, k) for t, k in [
            ('odom', Odometry), ('px4_pose', PoseStamped), ('image', Image),
            ('camera_info', CameraInfo), ('state', State), ('trigger', Bool)]}
        self.wait(lambda: all(pubs[t].get_num_connections() for t in
                             ('odom', 'image', 'camera_info', 'state', 'trigger')),
                  'FAST-LIO precision subscriptions')
        self.assertEqual(pubs['px4_pose'].get_num_connections(), 0)
        odom = Odometry()
        odom.header.frame_id = 'UAV1/camera_init'
        odom.pose.pose.position = Point(0.5, -2.5, 1.6)
        odom.pose.pose.orientation.w = math.cos(0.2)
        odom.pose.pose.orientation.z = math.sin(0.2)
        local = PoseStamped()
        local.pose.position = Point(3470, -925, 1.7)
        local.pose.orientation.w = 1.0
        state = State(connected=True, armed=True, mode='OFFBOARD')
        info = CameraInfo(width=640, height=480)
        info.K = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
        info.D = [0.] * 5
        blank = np.full((480, 640), 255, dtype=np.uint8)
        marked = blank.copy()
        marked[140:340, 220:420] = generate_aruco_marker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250), 37, 200)
        bridge = CvBridge()

        def drive(frames, visible, trigger=True):
            for _ in range(frames):
                pixels = (marked if visible else blank).copy()
                pixels[0, 0] = int(time.monotonic() * 1000) % 200
                image = bridge.cv2_to_imgmsg(pixels, encoding='mono8')
                local.pose.position.x += 2.0
                local.pose.position.y -= 1.0
                for topic, msg in [('odom', odom), ('px4_pose', local), ('state', state),
                                   ('camera_info', info), ('image', image)]:
                    msg.header.stamp = rospy.Time.now()
                    pubs[topic].publish(msg)
                pubs['trigger'].publish(Bool(data=trigger))
                time.sleep(0.05)

        drive(4, False, False)
        drive(8, False)
        hold = self.last('setpoint')
        self.assertIsNotNone(hold)
        self.assertEqual(hold.type_mask, 2552)
        self.assertAlmostEqual(hold.position.x, 0.5, places=4)
        self.assertAlmostEqual(hold.position.y, -2.5, places=4)
        self.assertAlmostEqual(hold.position.z, 1.6, places=4)
        self.assertAlmostEqual(hold.yaw, 0.4, places=4)
        drive(30, True)
        self.assertTrue(any(m.data.startswith('DESCEND_HIGH') for m in self.messages['stage']))
        odom.pose.pose.position = Point(0.3, -2.4, 1.5)
        drive(16, False)
        self.assertTrue(self.last('stage').data.startswith('REACQUIRE'))
        hold = self.last('setpoint')
        self.assertEqual(hold.type_mask, 2552)
        self.assertAlmostEqual(hold.position.x, 0.5, delta=.01)
        self.assertAlmostEqual(hold.position.y, -2.5, delta=.01)
        self.assertAlmostEqual(hold.position.z, 1.5, places=4)
        self.assertAlmostEqual(hold.yaw, 0.4, places=4)

    def test_ch10_stops_precision_mode_requests(self):
        import xml.etree.ElementTree as ET
        from test_precision_landing_node import PrecisionLandingNodeTest
        launch=ET.parse(Path(__file__).with_name('precision_landing_node.test'))
        ns=self.prefix+'/precision'
        for param in launch.getroot().find('node').findall('param'):
            rospy.set_param(ns+'/'+param.attrib['name'],yaml.safe_load(param.attrib['value']))
        rospy.set_param(ns+'/topics/rc_land_active',self.prefix+'/rc_land')
        node=roslib.packages.find_node('uav1_inplace_landing_test','uav1_test_precision_landing_node')[0]
        self.processes.append(subprocess.Popen([node,'__name:=precision','__ns:='+self.prefix],
            stdout=self.log,stderr=self.log,start_new_session=True))
        case=PrecisionLandingNodeTest('test_no_setpoint_before_trigger')
        case.setUp()
        self.addCleanup(case.doCleanups)
        self.addCleanup(case.tearDown)
        rc_land=self.publisher('rc_land',Bool,latch=True)
        self.wait(lambda:rc_land.get_num_connections()>0,'precision CH10 subscriber')
        case.publish_vehicle_context(armed=True,mode='OFFBOARD')
        case.publish_camera_info(valid=True)
        case.trigger_pub.publish(Bool(data=True))
        case.publish_centered_marker(marker_id=37,frames=14)
        self.assertTrue(any(x.startswith('FIXED_XY_DESCENT') for x in case._state_snapshot()))
        rc_land.publish(Bool(data=True));time.sleep(.2)
        count=len(case._setpoint_snapshot())
        case.publish_vehicle_context(armed=True,mode='OFFBOARD',pose_z=0.0)
        case.publish_camera_info(valid=True)
        case.trigger_pub.publish(Bool(data=True))
        case.publish_centered_marker(marker_id=37,frames=20)
        self.assertEqual(len(case._setpoint_snapshot()),count)
        self.assertEqual(case._mode_request_count(),0)

    def test_mission_handoff(self):
        base = Path(rospkg.RosPack().get_path('payload_mpc_controller'))
        def params(ns, data):
            for k,v in data.items():
                if isinstance(v,dict): params(ns+'/'+k,v)
                else: rospy.set_param(ns+'/'+k,v)
        for f in ('mpc.yaml', 'model.yaml'):
            params(self.prefix+'/controller',yaml.safe_load((base/'config'/f).read_text()))
        rospy.set_param(self.prefix+'/controller/precision_landing_enabled', True)
        mapping = {'~odom':'odom', '~force_attitude_odom':'odom', '~drone_imu/data':'imu',
                   '~mavros_local_pose':'local', '/mavros/state':'state',
                   '/mavros/extended_state':'extended', '/mavros/rc/in':'rc',
                   '/mavros/setpoint_raw/attitude':'control_attitude',
                   '~recovery_position_setpoint':'control_position', '~landing_target':'nmpc_target',
                   '~landing_state':'stage', '~rc_land_active':'rc_land',
                   '/mavros/set_mode':'set_mode'}
        node = roslib.packages.find_node('payload_mpc_controller','mpc_controller_node')[0]
        args = [node, '__name:=controller', '__ns:='+self.prefix]
        args += [k+':='+self.prefix+'/'+v for k,v in mapping.items()]
        self.processes.append(subprocess.Popen(args,stdout=self.log,stderr=self.log,start_new_session=True))
        rospy.set_param(self.prefix+'/arbiter/landing_via_mpc', True)
        self.launch('precision_landing','landing_setpoint_arbiter_node','arbiter',
                    trigger_topic='trigger', controller_position_input='control_position',
                    controller_attitude_input='control_attitude', landing_position_input='visual_target',
                    mavros_position_output='px4_position', mavros_attitude_output='px4_attitude',
                    mpc_target_topic='nmpc_target', active_topic='active', owner_topic='owner')
        for t,k in [('px4_position',PositionTarget),('px4_attitude',AttitudeTarget),
                    ('nmpc_target',PositionTarget),('rc_land',Bool),('owner',String),
                    ('controller/mpc/reference_trajectory',RosPath)]: self.subscribe(t,k)
        pubs = {t:self.publisher(t,k) for t,k in [('odom',Odometry),('local',PoseStamped),
                ('imu',Imu),('state',State),('extended',ExtendedState),('rc',RCIn),
                ('visual_target',PositionTarget),('trigger',Bool),('stage',String)]}
        self.wait(lambda: pubs['rc'].get_num_connections()>0 and
                  pubs['visual_target'].get_num_connections()>0, 'subscriptions')
        odom=Odometry(); odom.pose.pose.position=Point(1.2,-.7,1.5)
        odom.pose.pose.orientation.w=math.cos(.2); odom.pose.pose.orientation.z=math.sin(.2)
        local=PoseStamped(); local.pose.position=Point(3470,-925,1.7);local.pose.orientation.w=1
        imu=Imu();imu.orientation=odom.pose.pose.orientation;imu.linear_acceleration.z=9.795
        state=State(connected=True,armed=False,mode='POSCTL')
        ext=ExtendedState(landed_state=ExtendedState.LANDED_STATE_IN_AIR)
        rc=RCIn(channels=[1500]*10);rc.channels[7]=2000;rc.channels[9]=1000
        target=PositionTarget(coordinate_frame=1,type_mask=2552,yaw=.4)
        target.position=Point(1.2,-.7,1.5)
        trigger=False;stage='ALIGN'
        def drive(seconds,targets=True,px4_pose=False):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                now=rospy.Time.now()
                inputs=[('odom',odom),('imu',imu),('state',state),('extended',ext),('rc',rc)]
                if px4_pose:
                    local.pose.position.x += 1.0
                    local.pose.position.y -= 0.5
                    inputs.append(('local',local))
                for t,m in inputs:
                    m.header.stamp=now;pubs[t].publish(m)
                pubs['trigger'].publish(Bool(data=trigger));pubs['stage'].publish(String(data=stage))
                if targets:
                    target.header.stamp=now;pubs['visual_target'].publish(target)
                time.sleep(.01)
        drive(.8)
        self.assertIsNone(self.last('nmpc_target'), 'unreleased target reached NMPC')
        self.assertIsNone(self.last('px4_position'), 'vision bypassed NMPC')
        state.armed=True;state.mode='OFFBOARD';drive(.8)
        self.assertIsNotNone(self.last('px4_attitude'), 'normal task control absent')
        self.assertEqual(self.last('owner').data,'CONTROLLER')
        # 接管不需要 PX4 本地位姿；锁点直接使用 FAST-LIO 坐标。
        trigger=True;drive(.4)
        self.assertEqual(self.last('owner').data,'LANDING')
        self.assertIsNotNone(self.last('nmpc_target'))
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertAlmostEqual(ref.position.x,1.2,places=4)
        self.assertAlmostEqual(ref.position.y,-.7,places=4)
        self.assertAlmostEqual(2*math.atan2(ref.orientation.z,ref.orientation.w),.4,places=4)
        # 模拟丢码重新锁点，同时注入大幅漂移的 PX4 位姿。
        stage='REACQUIRE';target.position=Point(1.25,-.72,1.5);drive(.3,px4_pose=True)
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertAlmostEqual(ref.position.x,1.25,places=4)
        self.assertAlmostEqual(ref.position.y,-.72,places=4)
        stage='ALIGN';target.type_mask=2531;target.velocity.x=.06;drive(.5,px4_pose=True)
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertGreater(ref.position.x,1.21)
        self.assertAlmostEqual(ref.position.z,1.5,places=4)
        stage='DESCEND_HIGH';target.type_mask=2503;target.velocity.z=-.1;drive(.4)
        self.assertLess(self.last('controller/mpc/reference_trajectory').poses[0].pose.position.z,1.48)
        # 固定下降必须沿用保存的码中心目标，不得重置成 odom 当前 XY/Z。
        stage='FIXED_XY_DESCENT:STALE_IMAGE';target.type_mask=2552;target.position=Point(1.24,-.6,1.45);drive(.3)
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertAlmostEqual(ref.position.x,1.24,places=4)
        self.assertAlmostEqual(ref.position.y,-.6,places=4)
        self.assertAlmostEqual(ref.position.z,1.45,places=4)
        target.position.z-=.1;drive(.3)
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertAlmostEqual(ref.position.y,-.6,places=4)
        self.assertAlmostEqual(ref.position.z,1.35,places=4)
        drive(.6,False)
        self.assertAlmostEqual(self.last('controller/mpc/reference_trajectory').poses[0].pose.position.z,1.35,places=4)
        self.assertIsNone(self.last('px4_position'),'visual position leaked into PX4')
        rc.channels[9]=2000;target.position=Point(100,100,100);drive(.4)
        self.assertTrue(self.last('rc_land').data)
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertLess(ref.position.z,1.5)
        self.assertAlmostEqual(ref.position.x,1.2,places=4)
        state.mode='AUTO.LAND';drive(.2)
        count=len(self.messages['px4_attitude']);drive(.2)
        self.assertEqual(len(self.messages['px4_attitude']),count)


if __name__=='__main__':
    suite=unittest.TestSuite([
        MissionLandingTest('test_precision_uses_fastlio_for_acquire_and_marker_loss'),
        MissionLandingTest('test_mission_handoff'),
        MissionLandingTest('test_ch10_stops_precision_mode_requests')])
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
