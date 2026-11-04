#!/usr/bin/env python3
"""隔离 master，实际启动任务/路由节点；不启动相机、飞控或电机。"""
import sys
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
from std_msgs.msg import Bool, Float64, String


class BridgeTest(RcLandPriorityTest):
    def test_task_nmpc_precision(self):
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
        def drive(seconds,landing=False):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                for topic,msg in [('fast_lio/Odom_high_freq',odom),('livox/imu_mps2',imu),
                    ('mavros/local_position/pose',local),('mavros/state',state),
                    ('mavros/extended_state',ext),('mavros/rc/in',rc)]:
                    msg.header.stamp=rospy.Time.now();pubs[topic].publish(msg)
                pubs['landing/state'].publish(String(data='ALIGN' if landing else 'IDLE'))
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
        # CH10 后视觉参考不再覆盖人工降落参考；飞控坐标不参与精降目标转换。
        rc.channels[9]=2000;target.position=Point(99.,99.,99.);drive(.4,landing=True)
        ref=self.last('mpc_controller_node/mpc/reference_trajectory').poses[0].pose.position
        self.assertLess(ref.z,1.6)
        self.assertAlmostEqual(ref.x,1.2,places=4)

    def test_route_without_detections(self):
        params = dict(bridge_yaw=1.5707963267948966, entrance_x=10., entrance_y=20.,
                      floor_z=1., waypoints=[[0., 0., .6, 0.], [2., 0., .6, .15]],
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
        # 按实际航点依次到达；整个测试不发布任何检测消息。
        for i,y in enumerate([20.,22.,24.]):
            odom.pose.pose.position=Point(10.,y,1.6)
            odom.header.stamp=rospy.Time.now()
            pubs['odom'].publish(odom)
            if i < 2:
                self.wait(lambda: len(self.messages['goal']) >= i+2, 'next goal')
                self.assertIsNone(self.last('need_to_land'))
            else:
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
                             'test_task_nmpc_precision'])
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(not result.wasSuccessful())
