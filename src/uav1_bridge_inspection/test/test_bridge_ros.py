#!/usr/bin/env python3
"""隔离 master，实际启动任务/路由节点；不启动相机、飞控或电机。"""
import sys
import math
import subprocess
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'uav1_inplace_landing_test/test'))
from test_rc_land_priority_ros import RcLandPriorityTest
import rospy
import roslaunch
import roslib.packages
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import AttitudeTarget, ExtendedState, PositionTarget, RCIn, State
from mavros_msgs.srv import SetMode, SetModeResponse
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Log
from std_msgs.msg import Bool, Float64, String


class BridgeTest(RcLandPriorityTest):
    def test_task_nmpc_precision(self):
        controller_logs = []
        log_sub = rospy.Subscriber('/rosout', Log, controller_logs.append)
        self.addCleanup(log_sub.unregister)
        launch = str(Path(__file__).resolve().parents[1] / 'launch/control.launch')
        config = roslaunch.config.load_config_default([(launch, ['takeoff_z:=1.6'])], None)
        def mapped(name):
            return name.replace('/UAV1', self.prefix).replace('/bridge_landing_router', self.prefix+'/bridge_landing_router')
        for name, value in config.params.items():
            if name.startswith(('/UAV1/mpc_controller_node/', '/bridge_landing_router/')):
                v = value.value
                rospy.set_param(mapped(name), mapped(v) if isinstance(v,str) else v)
        for node in config.nodes:
            if node.name not in ('mpc_controller_node','bridge_landing_router'):
                continue
            executable = roslib.packages.find_node(node.package,node.type)[0]
            command=[executable,'__name:='+node.name,'__ns:='+self.prefix]
            command += [a+':='+mapped(b) for a,b in node.remap_args]
            self.processes.append(subprocess.Popen(command,stdout=self.log,stderr=self.log,start_new_session=True))
        service=rospy.Service(self.prefix+'/mavros/set_mode',SetMode,
                              lambda req:SetModeResponse(mode_sent=True))
        self.addCleanup(service.shutdown)
        self.subscribe('mavros/setpoint_raw/attitude',AttitudeTarget)
        self.subscribe('mpc_controller_node/mpc/reference_trajectory',RosPath)
        pubs={t:self.publisher(t,k) for t,k in [
            ('fast_lio/Odom_high_freq',Odometry),('livox/imu_mps2',Imu),
            ('mavros/state',State),('mavros/extended_state',ExtendedState),('mavros/rc/in',RCIn),
            ('mavros/local_position/pose',PoseStamped),('landing/state',String),
            ('bridge/precision_position',PositionTarget)]}
        self.wait(lambda: pubs['mavros/rc/in'].get_num_connections() and
                  pubs['bridge/precision_position'].get_num_connections(), 'task NMPC subscriptions')
        odom=Odometry();odom.pose.pose.orientation.w=1.;odom.pose.pose.position=Point(1.2,-.7,1.6)
        imu=Imu();imu.orientation.w=1.;imu.linear_acceleration.z=9.795
        local=PoseStamped();local.pose.orientation.w=1.;local.pose.position=Point(100.,200.,5.)
        state=State(connected=True,armed=False,mode='POSCTL')
        ext=ExtendedState(landed_state=ExtendedState.LANDED_STATE_IN_AIR)
        rc=RCIn(channels=[1500]*10);rc.channels[7]=1000;rc.channels[9]=1000
        target=PositionTarget(coordinate_frame=1,type_mask=2552)
        target.position=Point(1.25,-.72,1.5)
        landing_stage = 'ALIGN'
        def drive(seconds,landing=False):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                for topic,msg in [('fast_lio/Odom_high_freq',odom),('livox/imu_mps2',imu),
                    ('mavros/local_position/pose',local),('mavros/state',state),
                    ('mavros/extended_state',ext),('mavros/rc/in',rc)]:
                    msg.header.stamp=rospy.Time.now();pubs[topic].publish(msg)
                pubs['landing/state'].publish(String(data=landing_stage if landing else 'IDLE'))
                if landing:
                    target.header.stamp=rospy.Time.now();pubs['bridge/precision_position'].publish(target)
                time.sleep(.01)
        drive(.6)
        self.assertIsNotNone(self.last('mavros/setpoint_raw/attitude'))
        self.assertAlmostEqual(self.last('mavros/setpoint_raw/attitude').thrust,.01)
        rc.channels[7]=1500;state.armed=True;state.mode='OFFBOARD'
        drive(1.)
        self.assertGreater(self.last('mavros/setpoint_raw/attitude').thrust,.1)
        drive(.5,landing=True)
        ref=self.last('mpc_controller_node/mpc/reference_trajectory').poses[0].pose.position
        self.assertAlmostEqual(ref.x,1.25,places=4)
        self.assertAlmostEqual(ref.y,-.72,places=4)
        self.assertAlmostEqual(ref.z,1.5,places=4)
        landing_stage = 'FIXED_XY_DESCENT'
        drive(.5,landing=True)
        weight_log = '\n'.join(m.msg for m in controller_logs
                               if m.name == self.prefix+'/mpc_controller_node')
        locked_weights = 'active=1 Q_pos_xy=400.0 R_pitchroll=20.0'
        self.assertIn(locked_weights, weight_log)
        # 不能只验证曾经启用；设置位置参考后也必须保持加强权重。
        self.assertNotIn('active=0', weight_log.split(locked_weights, 1)[1])
        self.assertEqual(weight_log.count(locked_weights), 1)
        # CH10 后视觉参考不再覆盖人工降落参考；飞控坐标不参与精降目标转换。
        rc.channels[9]=2000;target.position=Point(99.,99.,99.);drive(.4,landing=True)
        ref=self.last('mpc_controller_node/mpc/reference_trajectory').poses[0].pose.position
        self.assertLess(ref.z,1.6)
        self.assertAlmostEqual(ref.x,1.2,places=4)
        state.mode = 'AUTO.LAND'
        drive(.3,landing=True)
        count = len(self.messages['mavros/setpoint_raw/attitude'])
        drive(.3,landing=True)
        self.assertEqual(len(self.messages['mavros/setpoint_raw/attitude']), count)

    def test_launch_ch8_route_and_landing(self):
        """使用真实 launch 接线和控制器 CH8 输出，逐点验证到达与精降交接。"""
        import cv2
        import numpy as np
        from cv_bridge import CvBridge
        from sensor_msgs.msg import CameraInfo, Image
        from test_precision_landing_node import generate_aruco_marker

        # 独立 master 上保留生产话题名，避免测试重映射掩盖接线错误。
        self.prefix = '/UAV1'
        launch = str(Path(__file__).resolve().parents[1] / 'launch/bridge.launch')
        config = roslaunch.config.load_config_default(
            [(launch, ['enable_detection:=false', 'enable_view:=false'])], None)
        for name, param in config.params.items():
            rospy.set_param(name, param.value)
        selected = ('mpc_controller_node', 'bridge_mission',
                    'precision_landing_node', 'bridge_landing_router')
        controller = None
        for node in config.nodes:
            if node.name not in selected:
                continue
            command = [roslib.packages.find_node(node.package, node.type)[0],
                       '__name:='+node.name, '__ns:='+node.namespace]
            command += [a+':='+b for a, b in node.remap_args]
            process = subprocess.Popen(command, stdout=self.log, stderr=self.log,
                                       start_new_session=True)
            self.processes.append(process)
            if node.name == 'mpc_controller_node':
                controller = process
        points = list(rospy.get_param('/bridge_mission/waypoints'))
        actions = list(points)
        points.append([rospy.get_param('/bridge_mission/'+key) for key in
                       ('landing_x', 'landing_y', 'landing_approach_z')])
        actions.append(points[-1])
        points = [point[:3] for point in points]
        self.subscribe('bridge/waypoint_turn', PoseStamped)
        approach_yaw = math.radians(rospy.get_param('/bridge_mission/landing_yaw_deg'))
        self.assertAlmostEqual(approach_yaw, -math.pi/2.)
        self.assertEqual(rospy.get_param('/UAV1/precision_landing_node/control/fixed_yaw_deg'), -90.)
        for topic, kind in [('planning/goal', PoseStamped), ('need_to_land', Bool),
                            ('landing/state', String), ('landing/nmpc_target', PositionTarget),
                            ('bridge/precision_position', PositionTarget)]:
            self.subscribe(topic, kind)
        pubs = {topic: self.publisher(topic, kind) for topic, kind in [
            ('fast_lio/Odom_high_freq', Odometry), ('livox/imu_mps2', Imu),
            ('mavros/state', State), ('mavros/extended_state', ExtendedState),
            ('mavros/rc/in', RCIn), ('down_camera/image_raw', Image),
            ('down_camera/camera_info', CameraInfo)]}
        self.wait(lambda: pubs['mavros/rc/in'].get_num_connections() and
                  pubs['fast_lio/Odom_high_freq'].get_num_connections() >= 4 and
                  pubs['down_camera/image_raw'].get_num_connections(), 'production subscriptions')
        odom = Odometry()
        odom.header.frame_id = 'UAV1/camera_init'
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.z = 2.
        state = State(connected=True, armed=False, mode='POSCTL')
        ext = ExtendedState(landed_state=ExtendedState.LANDED_STATE_IN_AIR)
        imu = Imu(); imu.orientation.w = 1.; imu.linear_acceleration.z = 9.795
        rc = RCIn(channels=[1500]*10); rc.channels[7] = 1000; rc.channels[9] = 1000
        info = CameraInfo(width=640, height=480)
        info.K = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
        info.D = [0.]*5
        pixels = np.full((480, 640), 255, dtype=np.uint8)
        pixels[211:269, 291:349] = generate_aruco_marker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 58)
        bridge = CvBridge()
        def drive(seconds):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                # 模拟新相机帧，避免同一张静态图片被精降冻结检测判为失效。
                pixels[0, 0] = int(time.monotonic()*1000) % 200
                image = bridge.cv2_to_imgmsg(pixels, 'mono8')
                for topic, msg in [('fast_lio/Odom_high_freq', odom), ('livox/imu_mps2', imu),
                                   ('mavros/state', state), ('mavros/extended_state', ext),
                                   ('mavros/rc/in', rc), ('down_camera/camera_info', info),
                                   ('down_camera/image_raw', image)]:
                    msg.header.stamp = rospy.Time.now()
                    pubs[topic].publish(msg)
                time.sleep(.02)
        drive(.5)
        state.armed = True; state.mode = 'OFFBOARD'; rc.channels[7] = 1500
        drive(.8)
        self.assertIsNone(self.last('planning/goal'))
        self.assertIsNone(self.last('need_to_land'))
        rc.channels[7] = 2000
        drive(.8)
        self.assertIsNotNone(self.last('planning/goal'), 'CH8 did not start bridge mission')
        self.assertEqual(len(self.messages['planning/goal']), 1)
        # 后续仅模拟逐点到达，不用跳变的定位数据驱动控制器求解。
        self.stop(controller)
        self.processes.remove(controller)
        radius = rospy.get_param('/bridge_mission/arrival_distance_m')
        speed = rospy.get_param('/bridge_mission/arrival_speed_mps')
        for index, point in enumerate(points):
            goal = self.last('planning/goal')
            self.assertEqual(goal.header.frame_id, 'UAV1/camera_init')
            self.assertEqual([goal.pose.position.x, goal.pose.position.y, goal.pose.position.z], point)
            expected_yaw = (approach_yaw if index == len(points)-1
                            else rospy.get_param('/bridge_mission/bridge_yaw'))
            q = goal.pose.orientation
            self.assertAlmostEqual(math.atan2(2*q.w*q.z, 1-2*q.z*q.z), expected_yaw)
            self.assertIsNone(self.last('need_to_land'))
            # 模拟跟随航点转向；精降应锁定到点时的右转航向而非桥洞航向。
            odom.pose.pose.orientation = q
            odom.pose.pose.position = Point(*point)
            odom.pose.pose.position.z += radius+.05
            drive(.12)
            self.assertEqual(len(self.messages['planning/goal']), index+1)
            self.assertIsNone(self.last('need_to_land'))
            odom.pose.pose.position = Point(*point)
            odom.twist.twist.linear.x = speed+.05
            drive(.12)
            self.assertEqual(len(self.messages['planning/goal']), index+1)
            self.assertIsNone(self.last('need_to_land'))
            odom.twist.twist.linear.x = 0.
            turn_deg = actions[index][4] if len(actions[index]) > 4 else 0.
            hold = actions[index][3] if len(actions[index]) > 3 else 0.
            if turn_deg:
                # 到点后才请求转向；未转到朝向时不能启动保持计时。
                drive(.3)
                self.assertEqual(len(self.messages['planning/goal']), index+1)
                turn = self.last('bridge/waypoint_turn')
                self.assertIsNotNone(turn)
                right_yaw = expected_yaw + math.radians(turn_deg)
                self.assertAlmostEqual(math.atan2(2*turn.pose.orientation.w*turn.pose.orientation.z,
                                      1-2*turn.pose.orientation.z**2), right_yaw)
                drive(hold+.1)
                self.assertEqual(len(self.messages['planning/goal']), index+1)
                odom.pose.pose.orientation = turn.pose.orientation
                drive(hold/2.)
                self.assertEqual(len(self.messages['planning/goal']), index+1)
                # 复现实飞：计时开始后位置、速度和航向短暂超限，仍应按时转回。
                odom.pose.pose.position.x += radius+.03
                odom.twist.twist.linear.x = speed+.10
                jitter_yaw = right_yaw + math.radians(6.)
                odom.pose.pose.orientation.z = math.sin(jitter_yaw/2.)
                odom.pose.pose.orientation.w = math.cos(jitter_yaw/2.)
                drive(hold/2.+.2)
                self.assertEqual(len(self.messages['planning/goal']), index+1)
                back = self.last('bridge/waypoint_turn')
                self.assertAlmostEqual(math.atan2(2*back.pose.orientation.w*back.pose.orientation.z,
                                      1-2*back.pose.orientation.z**2), expected_yaw)
                odom.pose.pose.orientation = back.pose.orientation
                drive(.2)
                # 转回后仍保留原到点距离/速度条件，不能带着超限位置直接切下一点。
                self.assertEqual(len(self.messages['planning/goal']), index+1)
                odom.pose.pose.position = Point(*point)
                odom.twist.twist.linear.x = 0.
            elif hold:
                drive(hold+.1)
            if index == len(points)-1:
                # 实际朝向在到点容差内偏3度，精降仍须使用指定-90度。
                odom.pose.pose.orientation.z = math.sin((approach_yaw+math.radians(3.))/2.)
                odom.pose.pose.orientation.w = math.cos((approach_yaw+math.radians(3.))/2.)
                drive(.6)  # 降落接近点摆正后连续稳定0.5秒。
            drive(.2)
            if index < len(points)-1:
                self.assertEqual(len(self.messages['planning/goal']), index+2)
                self.assertIsNone(self.last('need_to_land'))
        self.assertTrue(self.last('need_to_land').data)
        drive(3.)
        stages = [m.data for m in self.messages['landing/state']]
        self.assertTrue(any(s.startswith(('ALIGN', 'DESCEND_HIGH')) for s in stages), stages)
        target = self.last('landing/nmpc_target')
        self.assertIsNotNone(target, 'precision setpoint did not reach NMPC router output')
        self.assertAlmostEqual(target.yaw, approach_yaw, places=4)
        # ACQUIRE 保持最后航点位置；ALIGN/DESCEND 可输出速度，不能把被屏蔽的 position 当作目标。
        holds = [m for m in self.messages['landing/nmpc_target']
                 if not m.type_mask & (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY)]
        self.assertTrue(holds, 'missing position hold at landing approach point')
        self.assertAlmostEqual(holds[0].position.x, points[-1][0], delta=.01)
        self.assertAlmostEqual(holds[0].position.y, points[-1][1], delta=.01)
        # ROS 发布者会重新编号 header.seq；比较时间戳及实际控制字段。
        self.assertTrue(any(m.header.stamp == target.header.stamp and
                            m.type_mask == target.type_mask and
                            m.coordinate_frame == target.coordinate_frame and
                            m.position == target.position and m.velocity == target.velocity and
                            m.yaw == target.yaw
                            for m in self.messages['bridge/precision_position']),
                        'router changed precision reference')
        self.assertEqual(len(self.messages['planning/goal']), len(points))

    def test_route_without_detections(self):
        params = dict(bridge_yaw=1.5707963267948966,
                      waypoints=[[10., 20., 1.6, .6], [10., 22., 1.6]],
                      landing_x=10., landing_y=24., landing_approach_z=1.6,
                      landing_platform_delta_z=.1)
        for key, value in params.items():
            rospy.set_param(self.prefix+'/mission/'+key, value)
        self.launch('uav1_bridge_inspection', 'bridge_mission.py', 'mission')
        self.subscribe('goal', PoseStamped)
        self.subscribe('need_to_land', Bool)
        self.subscribe('platform_reference', Float64)
        pubs = {t:self.publisher(t,k) for t,k in [('start',PoseStamped),('state',State),('odom',Odometry)]}
        self.wait(lambda: all(p.get_num_connections() for p in self.handles), 'mission connections')
        odom = Odometry()
        odom.pose.pose.orientation.w=1.
        odom.pose.pose.position=Point(10.,19.,.2)
        pubs['state'].publish(State(connected=True,armed=False,mode='POSCTL'))
        pubs['odom'].publish(odom)
        self.wait(lambda: self.last('platform_reference') is not None, 'platform height')
        self.assertAlmostEqual(self.last('platform_reference').data,.3)
        self.assertIsNone(self.last('goal'))
        self.assertIsNone(self.last('need_to_land'))
        pubs['state'].publish(State(connected=True,armed=True,mode='OFFBOARD'))
        time.sleep(.1)
        pubs['start'].publish(PoseStamped())
        self.wait(lambda: self.last('goal') is not None, 'first goal')
        self.assertAlmostEqual(self.last('goal').pose.position.x,10.)
        self.assertAlmostEqual(self.last('goal').pose.position.y,20.)
        self.assertAlmostEqual(self.last('goal').pose.position.z,1.6)
        self.assertEqual(self.last('goal').header.frame_id,'UAV1/camera_init')
        # 按实际航点依次到达；整个测试不发布任何检测消息。
        for i,y in enumerate([20.,22.,24.]):
            goal=self.last('goal').pose.position
            self.assertAlmostEqual(goal.x,10.)
            self.assertAlmostEqual(goal.y,y)
            self.assertAlmostEqual(goal.z,1.6)
            odom.pose.pose.position=Point(10.,y,1.6)
            odom.header.stamp=rospy.Time.now()
            pubs['odom'].publish(odom)
            if i == 0:
                # 到点后保留当前目标，保持期内既不切换航点，也不触发精降。
                time.sleep(.3)
                self.assertEqual(len(self.messages['goal']), 1)
                self.assertIsNone(self.last('need_to_land'))
            if i < 2:
                self.wait(lambda: len(self.messages['goal']) >= i+2, 'next goal')
                self.assertIsNone(self.last('need_to_land'))
            else:
                # 到点但航向未摆正时不能精降；必须实际转正并连续保持0.5秒。
                self.subscribe('turn_goal', PoseStamped)
                time.sleep(.7)
                self.assertIsNone(self.last('need_to_land'))
                self.wait(lambda: self.last('turn_goal') is not None, 'landing turn request')
                target_q = self.last('turn_goal').pose.orientation
                self.assertAlmostEqual(2*math.atan2(target_q.z, target_q.w), -math.pi/2)
                odom.pose.pose.orientation = target_q
                pubs['odom'].publish(odom)
                time.sleep(.25)
                self.assertIsNone(self.last('need_to_land'))
                # 稳定计时中航向再次超限，计时必须重置。
                odom.pose.pose.orientation.z = math.sin((-math.pi/2-.30)/2)
                odom.pose.pose.orientation.w = math.cos((-math.pi/2-.30)/2)
                pubs['odom'].publish(odom)
                time.sleep(.15)
                odom.pose.pose.orientation = self.last('turn_goal').pose.orientation
                pubs['odom'].publish(odom)
                time.sleep(.3)
                self.assertIsNone(self.last('need_to_land'))
                self.wait(lambda: self.last('need_to_land') is not None, 'landing trigger')
        self.assertTrue(self.last('need_to_land').data)
        self.assertAlmostEqual(self.last('platform_reference').data,.3)
        count=len(self.messages['goal'])
        pubs['start'].publish(PoseStamped())
        time.sleep(.15)
        self.assertEqual(len(self.messages['goal']),count)

    def test_mission_router_handoff(self):
        rospy.set_param(self.prefix+'/router/mission_attitude_input',True)
        rospy.set_param(self.prefix+'/router/offboard_touchdown',True)
        rospy.set_param(self.prefix+'/router/odometry_topic',self.prefix+'/odom')
        self.launch('uav1_inplace_landing_test','landing_nmpc_test_router.py','router')
        for t,k in [('mavros_attitude',AttitudeTarget),('nmpc_target',PositionTarget)]:self.subscribe(t,k)
        pubs={t:self.publisher(t,k) for t,k in [('mavros_state',State),('nmpc_attitude',AttitudeTarget),
              ('landing_state',String),('landing_position',PositionTarget),('rc_land',Bool)]}
        self.wait(lambda: all(p.get_num_connections() for p in self.handles),'router connections')
        attitude=AttitudeTarget(type_mask=AttitudeTarget.IGNORE_ATTITUDE,thrust=.01)
        state=State(connected=True,armed=False,mode='POSCTL')
        pubs['mavros_state'].publish(state)
        attitude.header.stamp=rospy.Time.now();pubs['nmpc_attitude'].publish(attitude)
        self.wait(lambda:self.last('mavros_attitude') is not None,'prestream')
        self.assertAlmostEqual(self.last('mavros_attitude').thrust,.01)
        state.armed,state.mode=True,'OFFBOARD';pubs['mavros_state'].publish(state)
        time.sleep(.1)
        attitude.thrust=.5;attitude.header.stamp=rospy.Time.now();pubs['nmpc_attitude'].publish(attitude)
        self.wait(lambda:self.last('mavros_attitude').thrust>.4,'cruise output')
        self.assertIsNone(self.last('nmpc_target'))
        pubs['landing_state'].publish(String(data='ALIGN'))
        time.sleep(.1)
        target=PositionTarget(coordinate_frame=1,type_mask=2552)
        target.position=Point(10.,24.,1.6)
        target.header.stamp=rospy.Time.now();pubs['landing_position'].publish(target)
        self.wait(lambda:self.last('nmpc_target') is not None,'landing reference')
        self.assertAlmostEqual(self.last('nmpc_target').position.x,10.)
        state.mode='POSCTL';pubs['mavros_state'].publish(state);time.sleep(.1)
        count=len(self.messages['mavros_attitude'])
        attitude.header.stamp=rospy.Time.now();pubs['nmpc_attitude'].publish(attitude);time.sleep(.1)
        self.assertEqual(len(self.messages['mavros_attitude']),count)


if __name__=='__main__':
    suite=unittest.TestSuite(BridgeTest(name) for name in
                            ['test_route_without_detections','test_mission_router_handoff',
                             'test_task_nmpc_precision','test_launch_ch8_route_and_landing'])
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(not result.wasSuccessful())
