#!/usr/bin/env python3
"""隔离ROS master，真实NMPC节点配合模拟传感器验证精降测试交接，不连接飞控。"""
import math
import subprocess
import sys
import time
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_rc_land_priority_ros import RcLandPriorityTest
import rospy
import roslib.packages
import rospkg
import yaml
from geometry_msgs.msg import Accel, Point, PoseStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget, RCIn, State, ExtendedState, ESCStatus, ESCStatusItem
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String


class DescentTest(RcLandPriorityTest):
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
        node = roslib.packages.find_node('uav1_inplace_landing_test', 'uav1_test_precision_landing_node')[0]
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
        router = roslib.packages.find_node('uav1_inplace_landing_test', 'landing_nmpc_test_router.py')[0]
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
        node=roslib.packages.find_node('uav1_inplace_landing_test', 'uav1_test_precision_landing_node')[0]
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
        params(self.prefix + '/controller', yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/mpc_defaults.yaml').read_text()))
        for f in ('mpc.yaml', 'model.yaml'):
            params(self.prefix + '/controller', yaml.safe_load((base / 'config' / f).read_text()))
        rospy.set_param(self.prefix + '/controller/precision_landing_test_mode', True)
        rospy.set_param(self.prefix + '/controller/msg_timeout/state', 1.5)
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
                   '~precision_landing_test_target':'nmpc_target', '~rc_land_active':'rc_land',
                   '/mavros/esc_status':'esc_status'}
        node = roslib.packages.find_node('uav1_inplace_landing_test', 'uav1_test_mpc_controller_node')[0]
        args = [node, '__name:=controller', '__ns:=' + self.prefix]
        args += [k + ':=' + self.prefix + '/' + v for k, v in mapping.items()]
        self.processes.append(subprocess.Popen(args, stdout=self.log, stderr=self.log, start_new_session=True))
        router = roslib.packages.find_node('uav1_inplace_landing_test', 'landing_nmpc_test_router.py')[0]
        self.processes.append(subprocess.Popen([router,'__name:=router','__ns:='+self.prefix,
            'local_pose:='+self.prefix+'/local'],stdout=self.log,stderr=self.log,start_new_session=True))
        for t, kind in [('mavros_attitude',AttitudeTarget),('mavros_position',PositionTarget),
                        ('nmpc_attitude',AttitudeTarget),('owner',String),('rc_land',Bool),
                        ('controller/mpc/reference_trajectory',RosPath),
                        ('controller/mpc/force_applied',Accel)]: self.subscribe(t,kind)
        esc_pub = self.publisher('esc_status', ESCStatus)
        # 静止 IMU 与偏大的 RPM 推力构成向下外力，验证测试模式实际应用补偿。
        esc = ESCStatus(esc_status=[ESCStatusItem(rpm=16500) for _ in range(4)])
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
                esc.header.stamp = now
                esc_pub.publish(esc)
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
        self.assertIsNotNone(self.last('controller/mpc/force_applied'))
        self.assertLess(self.last('controller/mpc/force_applied').linear.z, -.2,
                        'independent NMPC test suppressed vertical disturbance compensation')
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
        self.assertAlmostEqual(self.last('controller/mpc/force_applied').linear.z, 0.0,
                               places=6, msg='CH10 descent must clear compensation')
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
    suite=unittest.TestSuite([DescentTest('test_repeated_marker_loss_does_not_raise_hold'), DescentTest('test_router_one_hz_state'), DescentTest('test_descent_handoff'), DescentTest('test_precision_auto_land_at_cutoff')])
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
