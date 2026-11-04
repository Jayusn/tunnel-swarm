#!/usr/bin/env python3
"""使用隔离 ROS master 和模拟相机/位姿测试丢码记忆与控制器超时下降。

source devel/setup.bash
/usr/bin/python3 -s src/uav1_inplace_landing_test/test/test_landing_memory_ros.py
可用 LANDING_TEST_PACKAGE / LANDING_TEST_EXECUTABLE 指定前机实际节点。
不连接实际飞控；仅向随机端口的 localhost master 发布模拟消息。
"""
import math
import os
import signal
import socket
import subprocess
import tempfile
import time
import unittest

import cv2
import numpy as np
import rosgraph
import roslib.packages
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from mavros_msgs.srv import SetMode, SetModeResponse
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String


class LandingMemoryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
        os.environ['ROS_HOSTNAME'] = '127.0.0.1'
        os.environ.pop('ROS_IP', None)
        cls.log = tempfile.NamedTemporaryFile(prefix='landing_memory_', suffix='.log', delete=False)
        print('隔离测试日志:', cls.log.name)
        cls.master = subprocess.Popen(['roscore', '-p', str(port)], stdout=cls.log,
                                      stderr=cls.log, start_new_session=True)
        for _ in range(100):
            try:
                rosgraph.Master('/memory_test').getPid()
                break
            except Exception:
                time.sleep(.1)
        else:
            cls.stop(cls.master)
            raise RuntimeError('isolated master failed')
        rospy.init_node('landing_memory_test', anonymous=True, disable_signals=True)

    @staticmethod
    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    @classmethod
    def tearDownClass(cls):
        rospy.signal_shutdown('test complete')
        cls.stop(cls.master)
        cls.log.close()

    def setUp(self):
        self.ns = '/' + self._testMethodName
        self.handles = []
        self.frames = True
        self.frozen = False
        self.have_marker = True
        self.pose_enabled = True
        self.trigger = False
        self.mode = 'OFFBOARD'
        self.landed = ExtendedState.LANDED_STATE_IN_AIR
        self.x, self.y, self.z = 10., 20., 3.
        self.seq = 0
        self.commands, self.states, self.errors, self.targets, self.requests = [], [], [], [], []
        self.bridge = CvBridge()
        self.pubs = {}
        for topic, cls in [('image', Image), ('info', CameraInfo), ('pose', PoseStamped),
                           ('state', State), ('extended', ExtendedState), ('trigger', Bool)]:
            self.pubs[topic] = rospy.Publisher(self.ns+'/'+topic, cls, queue_size=1)
            self.handles.append(self.pubs[topic])
        for topic, cls, dest in [('command', PositionTarget, self.commands),
                                 ('stage', String, self.states), ('error', PointStamped, self.errors),
                                 ('target', PoseStamped, self.targets)]:
            self.handles.append(rospy.Subscriber(self.ns+'/'+topic, cls, dest.append))
        self.service = rospy.Service(self.ns+'/mode', SetMode, self.mode_request)
        params = {'topics/image': 'image', 'topics/camera_info': 'info',
                  'topics/local_pose': 'pose', 'topics/mavros_state': 'state',
                  'topics/extended_state': 'extended', 'topics/trigger': 'trigger',
                  'topics/setpoint': 'command', 'topics/landing_state': 'stage',
                  'topics/error_xy': 'error', 'topics/target_pose': 'target',
                  'topics/set_mode_service': 'mode'}
        if getattr(self, 'use_fastlio', False):
            params['topics/odometry'] = 'odom'
            self.pubs['odom'] = rospy.Publisher(self.ns+'/odom', Odometry, queue_size=1)
            self.handles.append(self.pubs['odom'])
        for k, v in params.items():
            rospy.set_param(self.ns+'/node/'+k, self.ns+'/'+v)
        for k, v in {'safety/total_timeout_sec': 2., 'safety/reacquire_timeout_sec': 1.,
                     'safety/image_timeout_sec': .3, 'safety/pose_timeout_sec': .3,
                     'center_alignment/bias_xy_m': [.06, -.04], 'marker/size_m': .5,
                     'stages/fixed_xy_descent/descent_mps': .25}.items():
            rospy.set_param(self.ns+'/node/'+k, v)
        package = os.environ.get('LANDING_TEST_PACKAGE', 'uav1_inplace_landing_test')
        exe = os.environ.get('LANDING_TEST_EXECUTABLE', 'uav1_test_precision_landing_node')
        path = roslib.packages.find_node(package, exe)[0]
        self.node = subprocess.Popen([path, '__name:=node', '__ns:='+self.ns],
                                     stdout=self.log, stderr=self.log, start_new_session=True)
        self.addCleanup(self.cleanup)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.image = np.full((480, 640, 3), 255, np.uint8)
        marker = cv2.aruco.drawMarker(dictionary, 1, 120)
        self.image[180:300, 140:260] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        self.timer = rospy.Timer(rospy.Duration(.04), self.publish)
        self.wait(lambda: self.pubs['trigger'].get_num_connections() > 0, 'node connection')
        time.sleep(.35)

    def cleanup(self):
        self.timer.shutdown()
        self.stop(self.node)
        self.service.shutdown('test done')
        for handle in self.handles:
            handle.unregister()

    def mode_request(self, request):
        self.requests.append(request.custom_mode)
        return SetModeResponse(mode_sent=True)

    def publish(self, _):
        now = rospy.Time.now()
        pose = PoseStamped()
        pose.header.stamp = now
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.x, self.y, self.z
        pose.pose.orientation.z = pose.pose.orientation.w = math.sqrt(.5)  # yaw +90 deg
        if self.pose_enabled:
            if getattr(self, 'use_fastlio', False):
                odom = Odometry(header=pose.header)
                odom.pose.pose = pose.pose
                self.pubs['odom'].publish(odom)
                # 故意给飞控融合位姿加入偏移，验证码面记忆和末段交接只用里程计。
                pose = PoseStamped()
                pose.header.stamp = now
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = 100., 200., 30.
                pose.pose.orientation.w = 1.
            self.pubs['pose'].publish(pose)
        state = State(connected=True, armed=True, mode=self.mode)
        state.header.stamp = now
        self.pubs['state'].publish(state)
        extended = ExtendedState(landed_state=self.landed)
        extended.header.stamp = now
        self.pubs['extended'].publish(extended)
        self.pubs['trigger'].publish(Bool(data=self.trigger))
        if self.frames:
            info = CameraInfo(width=640, height=480, distortion_model='plumb_bob',
                              K=[600., 0., 320., 0., 600., 240., 0., 0., 1.], D=[0.]*5)
            info.header.stamp = now
            self.pubs['info'].publish(info)
            img = self.image.copy() if self.have_marker else np.full_like(self.image, 255)
            # 模拟传感器噪声；冻结测试中连同此像素一起保持不变。
            if not self.frozen:
                self.seq += 1
            img[0, 0, :] = self.seq % 200
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.stamp = now
            self.pubs['image'].publish(msg)

    def wait(self, predicate, reason):
        deadline = time.monotonic() + 6.
        while time.monotonic() < deadline:
            self.assertIsNone(self.node.poll(), 'node exited: '+self.log.name)
            if predicate():
                return
            time.sleep(.025)
        self.fail(reason + '; last states=' + str([s.data for s in self.states[-5:]]))

    def stage(self):
        return self.states[-1].data.split(':')[0] if self.states else ''

    def acquire(self):
        self.trigger = True
        self.wait(lambda: self.stage() == 'ALIGN' and self.errors, 'visual alignment')
        error = self.errors[-1].point
        # Body FLU -> world yaw +90, compensation already subtracted exactly once.
        expected = (10. - error.y, 20. + error.x)
        self.assertGreater(math.hypot(expected[0]-10., expected[1]-20.), .3)
        return expected

    def memory_scenario(self, frozen=False):
        expected = self.acquire()
        if frozen:
            self.frozen = True
            time.sleep(.38)
        else:
            self.frames = False
            time.sleep(.38)
        self.x += .2
        self.wait(lambda: self.commands and self.commands[-1].type_mask == 2552,
                  'saved target position command')
        cmd = self.commands[-1]
        self.assertAlmostEqual(cmd.position.x, expected[0], delta=.025)
        self.assertAlmostEqual(cmd.position.y, expected[1], delta=.025)
        self.assertAlmostEqual(cmd.position.z, 3., delta=.03)
        self.assertFalse(self.requests)
        self.x, self.y = expected
        self.wait(lambda: self.stage() == 'FIXED_XY_DESCENT', 'aligned odometry descent')
        self.wait(lambda: self.commands[-1].position.z < 2.95, 'descending ramp')
        self.assertFalse(self.requests, 'must not AUTO.LAND at timeout')
        # Camera remains unavailable; existing 0.3 m handoff still works.
        height = self.targets[-1].pose.position.z
        self.z = 3. - height + .25
        self.wait(lambda: self.requests, 'low-height handoff despite dead camera')
        self.assertEqual(self.requests[-1], 'AUTO.LAND')

    def test_stopped_camera_preserves_compensated_world_goal(self):
        self.memory_scenario()

    def test_repeated_pixels_with_new_timestamps_preserve_goal(self):
        self.memory_scenario(frozen=True)

    def test_missing_marker_preserves_goal_with_live_camera(self):
        expected = self.acquire()
        self.have_marker = False
        self.wait(lambda: self.commands[-1].type_mask == 2552, 'target memory')
        self.assertAlmostEqual(self.commands[-1].position.x, expected[0], delta=.025)
        self.wait(lambda: self.stage() == 'FIXED_XY_DESCENT', 'timeout starts controller')
        self.assertFalse(self.requests)

    def test_no_marker_timeout_descends_without_assuming_ground_zero(self):
        self.have_marker = False
        self.trigger = True
        self.wait(lambda: self.stage() == 'ACQUIRE', 'precheck')
        self.frames = False
        self.wait(lambda: self.stage() == 'FIXED_XY_DESCENT', 'timeout controller fallback')
        self.wait(lambda: self.commands[-1].position.z < 2.95, 'fallback ramp')
        self.assertAlmostEqual(self.commands[-1].position.x, 10.)
        self.assertFalse(self.requests)
        self.landed = ExtendedState.LANDED_STATE_ON_GROUND
        self.wait(lambda: self.requests, 'touchdown handoff')

    def test_pose_loss_still_aborts(self):
        self.acquire()
        self.frames = self.pose_enabled = False
        self.wait(lambda: self.stage() == 'ABORT_HOLD', 'pose stale abort')
        time.sleep(2.)
        self.assertEqual(self.stage(), 'ABORT_HOLD')
        self.assertFalse(self.requests)

    def test_offboard_loss_still_releases_control(self):
        self.acquire()
        self.mode = 'POSCTL'
        self.wait(lambda: self.stage() == 'PASSIVE_ABORT', 'manual takeover')
        count = len(self.commands)
        time.sleep(.4)
        self.assertLessEqual(len(self.commands), count+1)
        self.assertFalse(self.requests)


if __name__ == '__main__':
    unittest.main(verbosity=2)
