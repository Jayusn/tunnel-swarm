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
from mavros_msgs.msg import AttitudeTarget, PositionTarget, RCIn, State
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String


class DescentTest(RcLandPriorityTest):
    def test_descent_handoff(self):
        base = Path(rospkg.RosPack().get_path('payload_mpc_controller'))
        def params(ns, d):
            for k, v in d.items():
                if isinstance(v, dict): params(ns + '/' + k, v)
                else: rospy.set_param(ns + '/' + k, v)
        for f in ('mpc.yaml', 'model.yaml'):
            params(self.prefix + '/controller', yaml.safe_load((base / 'config' / f).read_text()))
        rospy.set_param(self.prefix + '/controller/precision_landing_test_mode', True)
        mapping = {'~odom':'odom', '~drone_imu/data':'imu', '~mavros_local_pose':'local',
                   '/mavros/state':'mavros_state', '/mavros/rc/in':'rc',
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
               ('mavros_state',State),('rc',RCIn),('landing_position',PositionTarget),('takeoff_position',PoseStamped),('landing_state',String)]}
        self.wait(lambda: pubs['rc'].get_num_connections() > 0 and pubs['landing_position'].get_num_connections()>0,'subscriptions')
        odom = Odometry(); odom.pose.pose.position = Point(1.2,-0.7,1.5)
        odom.pose.pose.orientation.w = math.cos(0.2); odom.pose.pose.orientation.z = math.sin(0.2)
        local = PoseStamped(); local.pose.position = Point(4,2,1.5); local.pose.orientation.w = 1
        imu = Imu(); imu.orientation=odom.pose.pose.orientation; imu.linear_acceleration.z=9.795
        state=State(connected=True,armed=False,mode='POSCTL')
        rc=RCIn(channels=[1500]*10); rc.channels[7]=2000; rc.channels[9]=1000
        target=PositionTarget(coordinate_frame=1,type_mask=2552,yaw=0.0)
        target.position=Point(4,2,1.5)
        stage='IDLE'
        takeoff=PoseStamped(); takeoff.pose.position=Point(4,2,3.2); takeoff.pose.orientation.w=1
        def drive(seconds, targets=True):
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                now=rospy.Time.now()
                for msg in (odom,local,imu,state,rc,target,takeoff): msg.header.stamp=now
                for topic,msg in [('odom',odom),('local',local),('imu',imu),('mavros_state',state),('rc',rc)]: pubs[topic].publish(msg)
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
        self.assertIsNotNone(self.last('mavros_attitude'), 'MPC did not take off')
        self.assertEqual(self.last('owner').data,'NMPC')
        ref=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertGreater(ref.position.z,1.5)
        self.assertLess(ref.position.z,1.65)
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
        rc.channels[9]=2000; drive(.5)
        self.assertTrue(self.last('rc_land').data,'CH10 did not latch')
        ref3=self.last('controller/mpc/reference_trajectory').poses[0].pose
        self.assertLess(ref3.position.z,1.5)
        state.mode='AUTO.LAND'; drive(.3)
        count=len(self.messages['mavros_attitude']); drive(.2)
        self.assertEqual(len(self.messages['mavros_attitude']),count,'output after AUTO.LAND')
        state.mode='OFFBOARD'; drive(.3)
        self.assertEqual(len(self.messages['mavros_attitude']),count,'unexpected reentry')


if __name__=='__main__':
    suite=unittest.TestSuite([DescentTest('test_descent_handoff')])
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
