#!/usr/bin/env python3
"""隔离ROS master，真实NMPC节点配合模拟传感器验证精降测试交接，不连接飞控。"""
import math
import subprocess
import sys
import time
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'autotrans_reference_bridge/test'))
from test_rc_land_priority_ros import RcLandPriorityTest
import rospy
import roslib.packages
import rospkg
import yaml
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget, RCIn, State, ExtendedState
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String


class DescentTest(RcLandPriorityTest):
    def test_fast_lio_takeoff_router(self):
        rospy.set_param(self.prefix+'/router/targets_in_fast_lio', True)
        node=roslib.packages.find_node('precision_landing','landing_nmpc_test_router.py')[0]
        self.processes.append(subprocess.Popen([node,'__name:=router','__ns:='+self.prefix],
            stdout=self.log,stderr=self.log,start_new_session=True))
        for t in ('mavros_position','nmpc_target'): self.subscribe(t,PositionTarget)
        pubs={t:self.publisher(t,k) for t,k in [('local_pose',PoseStamped),
            ('control_pose',PoseStamped),('takeoff_position',PoseStamped),('mavros_state',State)]}
        self.wait(lambda:all(p.get_num_connections() for p in pubs.values()),'router LIO inputs')
        local=PoseStamped();local.pose.position=Point(40,20,10);local.pose.orientation.w=1
        lio=PoseStamped();lio.pose.position=Point(1,2,1.5);lio.pose.orientation.w=1
        target=PoseStamped();target.pose.position=Point(1,2,3.2);target.pose.orientation.w=1
        state=State(connected=True,armed=False,mode='POSCTL')
        def drive(seconds):
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                for topic,msg in [('local_pose',local),('control_pose',lio),
                    ('mavros_state',state),('takeoff_position',target)]:
                    msg.header.stamp=rospy.Time.now();pubs[topic].publish(msg)
                time.sleep(.02)
        drive(.3)
        self.assertIsNone(self.last('nmpc_target'))
        self.assertAlmostEqual(self.last('mavros_position').position.z,10)
        state.armed=True;state.mode='OFFBOARD';drive(1.0)
        ref=self.last('nmpc_target').position
        self.assertAlmostEqual(ref.x,1);self.assertAlmostEqual(ref.y,2)
        self.assertGreater(ref.z,1.76);self.assertLess(ref.z,1.91)
        self.assertAlmostEqual(self.last('mavros_position').position.z,10)

    def test_fast_lio_mission_handoff(self):
        self.test_mission_mpc_handoff(fast_lio=True)

    def test_mission_mpc_handoff(self, fast_lio=False):
        base = Path(rospkg.RosPack().get_path('payload_mpc_controller'))
        def params(ns, values):
            for key, value in values.items():
                if isinstance(value, dict): params(ns+'/'+key, value)
                else: rospy.set_param(ns+'/'+key, value)
        for name in ('mpc.yaml', 'model.yaml'):
            params(self.prefix+'/controller', yaml.safe_load((base/'config'/name).read_text()))
        rospy.set_param(self.prefix+'/controller/precision_landing_mission_mode', True)
        rospy.set_param(self.prefix+'/controller/precision_target_in_fast_lio', fast_lio)
        rospy.set_param(self.prefix+'/arbiter/landing_via_mpc', True)
        self.launch('precision_landing', 'landing_setpoint_arbiter_node', 'arbiter',
            trigger_topic='trigger', rc_land_active_topic='rc_land',
            controller_position_input='recovery', controller_attitude_input='attitude',
            landing_position_input='landing', landing_mpc_output='target',
            mavros_position_output='position_out', mavros_attitude_output='attitude_out',
            active_topic='active', owner_topic='owner')
        mapping = {'~odom':'odom', '~drone_imu/data':'imu', '~mavros_local_pose':'local',
            '/mavros/state':'state', '/mavros/rc/in':'rc',
            '/mavros/setpoint_raw/attitude':'attitude',
            '~recovery_position_setpoint':'recovery', '~precision_landing_test_target':'target',
            '~rc_land_active':'rc_land'}
        node = roslib.packages.find_node('payload_mpc_controller', 'mpc_controller_node')[0]
        self.processes.append(subprocess.Popen([node, '__name:=controller', '__ns:='+self.prefix]+
            [key+':='+self.prefix+'/'+value for key,value in mapping.items()],
            stdout=self.log, stderr=self.log, start_new_session=True))
        for topic, kind in [('target',PositionTarget),('position_out',PositionTarget),
            ('attitude_out',AttitudeTarget),('owner',String),('rc_land',Bool),
            ('controller/mpc/reference_trajectory',RosPath)]: self.subscribe(topic,kind)
        pubs={topic:self.publisher(topic,kind) for topic,kind in [
            ('odom',Odometry),('imu',Imu),('local',PoseStamped),('state',State),
            ('rc',RCIn),('landing',PositionTarget),('trigger',Bool)]}
        self.wait(lambda: all(pub.get_num_connections() for pub in pubs.values()), 'mission subscriptions')
        odom=Odometry(); odom.pose.pose.position=Point(1.2,-.7,1.5)
        odom.pose.pose.orientation.w=math.cos(.2); odom.pose.pose.orientation.z=math.sin(.2)
        local=PoseStamped(); local.pose.position=Point(4,2,1.5); local.pose.orientation.w=1
        imu=Imu(); imu.orientation=odom.pose.pose.orientation; imu.linear_acceleration.z=9.795
        state=State(connected=True,armed=True,mode='OFFBOARD')
        rc=RCIn(channels=[1500]*10); rc.channels[7]=2000; rc.channels[9]=1000
        target=PositionTarget(coordinate_frame=1,type_mask=2552,yaw=0)
        target.position=Point(1.2,-.7,1.2) if fast_lio else Point(4,2,1.2)
        target.yaw=.4 if fast_lio else 0.0
        def drive(seconds):
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                for topic,msg in [('odom',odom),('imu',imu),('local',local),('state',state),('rc',rc),('landing',target)]:
                    msg.header.stamp=rospy.Time.now(); pubs[topic].publish(msg)
                time.sleep(.01)
        drive(1.3)
        self.assertFalse(self.messages['target'], 'landing forwarded before mission trigger')
        self.assertTrue(self.messages['attitude_out'], 'normal controller output blocked')
        pubs['trigger'].publish(Bool(data=True)); drive(.5)
        self.assertEqual(self.last('owner').data,'LANDING')
        reference=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertAlmostEqual(reference.position.x,1.2,places=4)
        self.assertAlmostEqual(reference.position.y,-.7,places=4)
        self.assertAlmostEqual(reference.position.z,1.2,places=4)
        self.assertAlmostEqual(2*math.atan2(reference.orientation.z,reference.orientation.w),.4,places=4)
        self.assertFalse(self.messages['position_out'], 'precision target bypassed MPC')
        count=len(self.messages['attitude_out']); drive(.2)
        self.assertGreater(len(self.messages['attitude_out']),count)
        target.type_mask=2531; target.velocity.x=.1; drive(.2)
        reference=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertGreater(reference.position.x,1.2)
        self.assertAlmostEqual(reference.position.z,1.2,places=4)
        target.type_mask=2503; target.velocity.z=-.1; drive(.2)
        self.assertLess(self.last('controller/mpc/reference_trajectory').poses[0].pose.position.z,1.5)
        target.type_mask=2552; target.position.z=1.1; drive(.2)
        self.assertAlmostEqual(self.last('controller/mpc/reference_trajectory').poses[0].pose.position.z,1.1,places=4)
        rc.channels[9]=2000; drive(.5)
        self.assertTrue(self.last('rc_land').data)
        self.assertEqual(self.last('owner').data,'RC_LAND')
        count=len(self.messages['target']); target.position=Point(20,20,20); drive(.2)
        self.assertEqual(len(self.messages['target']),count,'CH10 did not stop precision targets')
        reference=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertAlmostEqual(reference.position.x,1.2,places=4)
        self.assertAlmostEqual(reference.position.y,-.7,places=4)
        self.assertLess(reference.position.z,1.5)
        state.mode='AUTO.LAND'; drive(.2)
        count=len(self.messages['attitude_out']); drive(.2)
        self.assertEqual(len(self.messages['attitude_out']),count)
        state.mode='OFFBOARD'; drive(.2)
        self.assertEqual(len(self.messages['attitude_out']),count,'unexpected mission reentry')

    def test_repeated_marker_loss_does_not_raise_hold(self):
        import xml.etree.ElementTree as ET
        import numpy as np
        from test_precision_landing_node import PrecisionLandingNodeTest
        launch = ET.parse(Path(__file__).with_name('precision_landing_node.test'))
        ns = self.prefix + '/precision'
        for param in launch.getroot().find('node').findall('param'):
            rospy.set_param(ns+'/'+param.attrib['name'], yaml.safe_load(param.attrib['value']))
        for key in ('state_machine/high_height_m', 'state_machine/auto_land_height_m'):
            rospy.set_param(ns+'/'+key, 1.2)
        node = roslib.packages.find_node('precision_landing', 'precision_landing_node')[0]
        self.processes.append(subprocess.Popen(
            [node, '__name:=precision', '__ns:='+self.prefix],
            stdout=self.log, stderr=self.log, start_new_session=True))
        case = PrecisionLandingNodeTest('test_no_setpoint_before_trigger')
        case.setUp()
        self.addCleanup(case.doCleanups)
        self.addCleanup(case.tearDown)

        def context(z):
            case.publish_vehicle_context(armed=True, mode='OFFBOARD', pose_z=z)
            case.publish_camera_info(valid=True)

        def lose_marker(z):
            context(z)
            for _ in range(14):
                msg = case.bridge.cv2_to_imgmsg(np.full((480, 640), 255, dtype=np.uint8), encoding='mono8')
                msg.header.stamp = rospy.Time.now()
                case.image_pub.publish(msg)
                rospy.sleep(.05)
            self.assertTrue(case._state_snapshot()[-1].startswith('REACQUIRE'))
            hold = case._setpoint_snapshot()[-1]
            self.assertEqual(hold.type_mask, 2552)
            return hold.position.z

        context(2.0)
        case.trigger_pub.publish(Bool(data=True))
        case.publish_centered_marker(marker_id=37, frames=14)
        self.assertTrue(case._state_snapshot()[-1].startswith('DESCEND_HIGH'))
        context(1.5)
        case.publish_centered_marker(marker_id=37, frames=4)
        first_hold = lose_marker(1.5)
        self.assertAlmostEqual(first_hold, 1.5, places=4)
        # 飞机向上偏移，重识别后仍需对准，再次丢码不能抬高悬停目标。
        context(1.7)
        case.publish_centered_marker(marker_id=37, frames=5, center_x=400)
        self.assertTrue(case._state_snapshot()[-1].startswith('ALIGN'))
        self.assertAlmostEqual(case._setpoint_snapshot()[-1].position.z, first_hold, places=4)
        self.assertAlmostEqual(lose_marker(1.8), first_hold, places=4)
        # 即使重新进入下降阶段，也不能用上浮后的定位覆盖原来的高度上限。
        context(1.8)
        case.publish_centered_marker(marker_id=37, frames=10)
        self.assertTrue(case._state_snapshot()[-1].startswith('DESCEND_HIGH'))
        self.assertAlmostEqual(lose_marker(1.9), first_hold, places=4)
        context(1.3)
        case.publish_centered_marker(marker_id=37, frames=10)
        self.assertAlmostEqual(lose_marker(1.3), 1.3, places=4)
        self.assertEqual(case._mode_request_count(), 0)

    def test_router_one_hz_state(self):
        router = roslib.packages.find_node('precision_landing', 'landing_nmpc_test_router.py')[0]
        self.processes.append(subprocess.Popen(
            [router, '__name:=router', '__ns:='+self.prefix],
            stdout=self.log, stderr=self.log, start_new_session=True))
        for topic, kind in [('nmpc_target', PositionTarget), ('mavros_attitude', AttitudeTarget),
                            ('mavros_position', PositionTarget), ('owner', String)]:
            self.subscribe(topic, kind)
        pubs = {t: self.publisher(t, k) for t, k in [
            ('mavros_state', State), ('local_pose', PoseStamped),
            ('takeoff_position', PoseStamped), ('landing_position', PositionTarget),
            ('landing_state', String), ('nmpc_attitude', AttitudeTarget),
            ('nmpc_recovery', PositionTarget)]}
        self.wait(lambda: all(p.get_num_connections() for p in pubs.values()), 'router subscriptions')
        state = State(connected=True, armed=True, mode='OFFBOARD')
        pose = PoseStamped(); pose.pose.orientation.w = 1
        target = PositionTarget(coordinate_frame=1, type_mask=2552)
        attitude = AttitudeTarget(type_mask=AttitudeTarget.IGNORE_ATTITUDE, thrust=.45)
        next_state = 0.0

        def drive(seconds, stage='IDLE', heartbeat=True, recovery=False):
            nonlocal next_state
            until = time.monotonic()+seconds
            while time.monotonic() < until:
                now = rospy.Time.now()
                if heartbeat and time.monotonic() >= next_state:
                    state.header.stamp = now
                    pubs['mavros_state'].publish(state)
                    next_state = time.monotonic()+1.0
                for msg in (pose, target, attitude): msg.header.stamp = now
                pubs['local_pose'].publish(pose)
                pubs['landing_state'].publish(String(data=stage))
                if stage == 'IDLE': pubs['takeoff_position'].publish(pose)
                else: pubs['landing_position'].publish(target)
                if recovery: pubs['nmpc_recovery'].publish(target)
                else: pubs['nmpc_attitude'].publish(attitude)
                time.sleep(.02)

        # 实机 State 为 1 Hz；目标和控制输出必须跨过每次心跳的后半秒。
        for stage in ('IDLE', 'ALIGN', 'FIXED_XY_DESCENT'):
            since = rospy.Time.now().to_sec()
            drive(2.2, stage)
            for topic in ('nmpc_target', 'mavros_attitude'):
                stamps = [m.header.stamp.to_sec() for m in self.messages[topic]
                          if m.header.stamp.to_sec() >= since]
                self.assertGreater(len(stamps), 70, (stage, topic))
                self.assertLess(max(b-a for a, b in zip(stamps, stamps[1:])), .2,
                                (stage, topic, 'periodic heartbeat output gap'))
        since = rospy.Time.now().to_sec()
        drive(2.2, 'FIXED_XY_DESCENT', recovery=True)
        stamps = [m.header.stamp.to_sec() for m in self.messages['mavros_position']
                  if m.header.stamp.to_sec() >= since]
        self.assertGreater(len(stamps), 70)
        self.assertLess(max(b-a for a, b in zip(stamps, stamps[1:])), .2)
        # 真正断流仍需停止转发；恢复有效状态后才继续。
        drive(1.7, 'FIXED_XY_DESCENT', heartbeat=False)
        counts = {t: len(self.messages[t]) for t in ('nmpc_target', 'mavros_attitude', 'mavros_position')}
        drive(.3, 'FIXED_XY_DESCENT', heartbeat=False)
        drive(.3, 'FIXED_XY_DESCENT', heartbeat=False, recovery=True)
        self.assertEqual(counts, {t: len(self.messages[t]) for t in counts})
        drive(.3, 'FIXED_XY_DESCENT')
        self.assertGreater(len(self.messages['mavros_attitude']), counts['mavros_attitude'])
        state.mode = 'POSCTL'; next_state = 0.0
        drive(.15, 'FIXED_XY_DESCENT')
        count = len(self.messages['mavros_attitude'])
        drive(.3, 'FIXED_XY_DESCENT')
        self.assertEqual(len(self.messages['mavros_attitude']), count)
        self.assertEqual(self.last('owner').data, 'STOPPED')

    def test_precision_auto_land_at_cutoff(self):
        import xml.etree.ElementTree as ET
        from test_precision_landing_node import PrecisionLandingNodeTest
        launch = ET.parse(Path(__file__).with_name('precision_landing_node.test'))
        ns = self.prefix + '/precision'
        for param in launch.getroot().find('node').findall('param'):
            rospy.set_param(ns+'/'+param.attrib['name'], yaml.safe_load(param.attrib['value']))
        rospy.set_param(ns+'/auto_land_at_cutoff', True)
        node=roslib.packages.find_node('precision_landing','precision_landing_node')[0]
        self.processes.append(subprocess.Popen([node,'__name:=precision','__ns:='+self.prefix],
            stdout=self.log,stderr=self.log,start_new_session=True))
        case=PrecisionLandingNodeTest('test_falling_trigger_keeps_fixed_descent_active')
        case.setUp()
        self.addCleanup(case.doCleanups)
        self.addCleanup(case.tearDown)
        case.publish_vehicle_context(armed=True,mode='OFFBOARD')
        case.publish_camera_info(valid=True)
        case.trigger_pub.publish(Bool(data=True))
        case.publish_centered_marker(marker_id=37,frames=14)
        self.assertTrue(any(x.startswith('FIXED_XY_DESCENT') for x in case._state_snapshot()))
        case.publish_vehicle_context(armed=True,mode='OFFBOARD',pose_z=0.0)
        case.publish_camera_info(valid=True)
        case.publish_centered_marker(marker_id=37,frames=20)
        self.assertTrue(case._state_snapshot()[-1].startswith('REQUEST_AUTO_LAND'))
        self.assertGreater(case._mode_request_count(), 0, 'no AUTO.LAND below cutoff')

    def test_descent_handoff(self):
        base = Path(rospkg.RosPack().get_path('payload_mpc_controller'))
        def params(ns, d):
            for k, v in d.items():
                if isinstance(v, dict): params(ns + '/' + k, v)
                else: rospy.set_param(ns + '/' + k, v)
        for f in ('mpc.yaml', 'model.yaml'):
            params(self.prefix + '/controller', yaml.safe_load((base / 'config' / f).read_text()))
        rospy.set_param(self.prefix + '/controller/precision_landing_test_mode', True)
        rospy.set_param(self.prefix + '/controller/land/timeout', 8.0)
        disarm_calls=[]; mode_calls=[]
        disarm_service=rospy.Service(self.prefix+'/arming', CommandBool,
            lambda req: (disarm_calls.append(req.value) or CommandBoolResponse(success=False, result=1)))
        mode_service=rospy.Service(self.prefix+'/set_mode',SetMode,
            lambda req: (mode_calls.append(req.custom_mode) or SetModeResponse(mode_sent=True)))
        self.addCleanup(disarm_service.shutdown)
        self.addCleanup(mode_service.shutdown)
        mapping = {'~odom':'odom', '~drone_imu/data':'imu', '~mavros_local_pose':'local',
                   '/mavros/set_mode':'set_mode', '/mavros/extended_state':'extended_state', '/mavros/state':'mavros_state', '/mavros/rc/in':'rc',
                   '/mavros/setpoint_raw/attitude':'nmpc_attitude',
                   '~recovery_position_setpoint':'nmpc_recovery',
                   '~precision_landing_test_target':'nmpc_target', '~rc_land_active':'rc_land'}
        node = roslib.packages.find_node('payload_mpc_controller', 'mpc_controller_node')[0]
        args = [node, '__name:=controller', '__ns:=' + self.prefix]
        args += [k + ':=' + self.prefix + '/' + v for k, v in mapping.items()]
        self.processes.append(subprocess.Popen(args, stdout=self.log, stderr=self.log, start_new_session=True))
        router = roslib.packages.find_node('precision_landing','landing_nmpc_test_router.py')[0]
        self.processes.append(subprocess.Popen([router,'__name:=router','__ns:='+self.prefix,
            'local_pose:='+self.prefix+'/local'],stdout=self.log,stderr=self.log,start_new_session=True))
        for t, kind in [('mavros_attitude',AttitudeTarget),('mavros_position',PositionTarget),
                        ('nmpc_attitude',AttitudeTarget),('owner',String),('rc_land',Bool),
                        ('controller/mpc/reference_trajectory',RosPath)]: self.subscribe(t,kind)
        pubs = {t:self.publisher(t,k) for t,k in [('odom',Odometry),('imu',Imu),('local',PoseStamped),
               ('extended_state',ExtendedState),('mavros_state',State),('rc',RCIn),('landing_position',PositionTarget),('takeoff_position',PoseStamped),('landing_state',String)]}
        self.wait(lambda: pubs['rc'].get_num_connections() > 0 and pubs['landing_position'].get_num_connections()>0,'subscriptions')
        odom = Odometry(); odom.pose.pose.position = Point(1.2,-0.7,1.5)
        odom.pose.pose.orientation.w = math.cos(0.2); odom.pose.pose.orientation.z = math.sin(0.2)
        local = PoseStamped(); local.pose.position = Point(4,2,1.5); local.pose.orientation.w = 1
        imu = Imu(); imu.orientation=odom.pose.pose.orientation; imu.linear_acceleration.z=9.795
        state=State(connected=True,armed=False,mode='POSCTL')
        rc=RCIn(channels=[1500]*10); rc.channels[7]=2000; rc.channels[9]=1000
        target=PositionTarget(coordinate_frame=1,type_mask=2552,yaw=0.0)
        target.position=Point(4,2,1.5)
        ext=ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
        stage='IDLE'
        takeoff=PoseStamped(); takeoff.pose.position=Point(4,2,3.2); takeoff.pose.orientation.w=1
        next_state = 0.0
        last_state = None
        def drive(seconds, targets=True):
            nonlocal next_state, last_state
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                now=rospy.Time.now()
                for msg in (odom,local,imu,state,rc,target,takeoff,ext): msg.header.stamp=now
                current_state = (state.connected, state.armed, state.mode)
                if current_state != last_state or time.monotonic() >= next_state:
                    pubs['mavros_state'].publish(state)
                    last_state = current_state
                    next_state = time.monotonic()+1.0
                for topic,msg in [('odom',odom),('local',local),('imu',imu),('rc',rc),('extended_state',ext)]: pubs[topic].publish(msg)
                pubs['landing_state'].publish(String(data=stage))
                if targets:
                    if stage=='IDLE': pubs['takeoff_position'].publish(takeoff)
                    else: pubs['landing_position'].publish(target)
                time.sleep(.01)
        drive(.7)
        self.assertIsNone(self.last('nmpc_attitude'), 'NMPC output before arming')
        self.assertIsNotNone(self.last('mavros_position'))
        self.assertAlmostEqual(self.last('mavros_position').position.z,1.5,places=4)
        state.armed=True; state.mode='OFFBOARD'; drive(.8)
        self.assertFalse(disarm_calls, 'disarm during takeoff on-ground state')
        ext.landed_state=ExtendedState.LANDED_STATE_IN_AIR; drive(.2)
        self.assertIsNotNone(self.last('mavros_attitude'), 'MPC did not take off')
        self.assertEqual(self.last('owner').data,'NMPC')
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertGreater(ref.position.z,1.5)
        # 1.70米目标分5秒爬升；约1秒应前进0.34米，允许ROS调度误差。
        self.assertGreater(ref.position.z,1.77)
        self.assertLess(ref.position.z,1.91)
        drive(4.3)
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertAlmostEqual(ref.position.z,3.2,places=4)
        count=len(self.messages['mavros_position'])
        stage='ALIGN'; target.type_mask=2531; target.position.z=1.5
        target.velocity.x=.06; target.velocity.y=-.02; drive(.4)
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertGreater(ref.position.x,1.2)
        self.assertAlmostEqual(ref.position.z,1.5,places=4)
        stage='DESCEND_HIGH'; target.type_mask=2503; target.velocity.z=-.1; drive(.4)
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertLess(ref.position.z,1.48)
        self.assertEqual(len(self.messages['mavros_position']),count, 'PX4 stream during visual alignment')
        stage='FIXED_XY_DESCENT'; target.type_mask=2552
        target.position=Point(4.04,1.98,1.45); drive(.4)
        self.assertEqual(len(self.messages['mavros_position']),count,'PX4 precision stream leaked after NMPC takeover')
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertAlmostEqual(ref.position.x,1.2 + .04*math.cos(.4) + .02*math.sin(.4),places=4)
        self.assertAlmostEqual(ref.position.y,-.7 + .04*math.sin(.4) - .02*math.cos(.4),places=4)
        self.assertAlmostEqual(ref.position.z,1.45,places=4)
        self.assertAlmostEqual(2*math.atan2(ref.orientation.z,ref.orientation.w),.4,places=4)
        drive(.6,False)
        ref2=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertAlmostEqual(ref2.position.z,ref.position.z,places=4)
        odom.pose.pose.position.z = 2.5
        local.pose.position.z = 2.5
        rc.channels[9]=2000; drive(.5)
        self.assertTrue(self.last('rc_land').data,'CH10 did not latch')
        ref3=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertLess(ref3.position.z,2.5)
        self.assertFalse(mode_calls, 'normal CH10 requested AUTO.LAND')
        self.assertFalse(disarm_calls, 'disarm while airborne')
        # CH10到起飞平面上方0.2米，触发近地AUTO.LAND。
        odom.pose.pose.position.z = 1.7
        local.pose.position.z = 1.7
        drive(.6)
        self.assertIn('AUTO.LAND', mode_calls)
        self.assertFalse(disarm_calls)
        state.mode='AUTO.LAND'; drive(.2)
        self.assertEqual(self.last('owner').data,'STOPPED')
        count=len(self.messages['mavros_attitude']); drive(.2)
        self.assertEqual(len(self.messages['mavros_attitude']),count,'output after AUTO.LAND')
        state.mode='OFFBOARD'; drive(.3)
        self.assertEqual(len(self.messages['mavros_attitude']),count,'unexpected reentry')




if __name__=='__main__':
    suite=unittest.TestSuite([DescentTest('test_fast_lio_takeoff_router'), DescentTest('test_fast_lio_mission_handoff'), DescentTest('test_mission_mpc_handoff'), DescentTest('test_repeated_marker_loss_does_not_raise_hold'), DescentTest('test_router_one_hz_state'), DescentTest('test_descent_handoff'), DescentTest('test_precision_auto_land_at_cutoff')])
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
