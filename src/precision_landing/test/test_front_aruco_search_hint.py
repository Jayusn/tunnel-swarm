#!/usr/bin/env python3
"""真实图像链路：深度拒绝保留搜索线索，恢复一致后才输出确认平台。"""
import time
import unittest

import cv2
import numpy as np
import rospy
import rostest
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from precision_landing.msg import LandingPlatformArray


class FrontArucoSearchHintTest(unittest.TestCase):
    def test_depth_rejected_position_survives_for_verification(self):
        self.confirmed = []
        self.rough = []
        confirmed_sub = rospy.Subscriber('/front_test/confirmed', LandingPlatformArray,
                                         lambda m: self.confirmed.append(m))
        rough_sub = rospy.Subscriber('/front_test/rough', LandingPlatformArray,
                                     lambda m: self.rough.append(m))
        image_pub = rospy.Publisher('/front_test/image', Image, queue_size=1)
        depth_pub = rospy.Publisher('/front_test/depth', Image, queue_size=1)
        info_pub = rospy.Publisher('/front_test/info', CameraInfo, queue_size=1, latch=True)
        odom_pub = rospy.Publisher('/front_test/odom', Odometry, queue_size=10)
        stage_pub = rospy.Publisher('/front_test/stage', String, queue_size=1, latch=True)
        state_pub = rospy.Publisher('/front_test/state', String, queue_size=1, latch=True)
        deadline = time.monotonic() + 5
        while not all(p.get_num_connections() for p in
                      (image_pub, depth_pub, info_pub, odom_pub, stage_pub, state_pub)):
            self.assertLess(time.monotonic(), deadline)
            rospy.sleep(.05)
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'front_test_body'
        transform.child_frame_id = 'front_test_camera'
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        info = CameraInfo()
        info.width, info.height = 640, 480
        info.K = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
        info.D = [0.] * 5
        info.distortion_model = 'plumb_bob'
        info_pub.publish(info)
        stage_pub.publish(String(data='SEARCH_OUTSIDE_LANDING'))
        state_pub.publish(String(data='FRONT_ARUCO_VERIFY_APPROACH'))
        rospy.sleep(.2)
        marker = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250), 2, 200)
        gray = np.full((480, 640), 255, dtype=np.uint8)
        gray[140:340, 220:420] = marker
        color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        bridge = CvBridge()

        def frames(depth, count):
            for _ in range(count):
                stamp = rospy.Time.now()
                odom = Odometry()
                odom.header.stamp = stamp
                odom.pose.pose.orientation.w = 1.
                odom_pub.publish(odom)
                depth_msg = bridge.cv2_to_imgmsg(np.full((480, 640), depth, np.float32), '32FC1')
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = 'front_test_camera'
                depth_pub.publish(depth_msg)
                rospy.sleep(.015)
                image = bridge.cv2_to_imgmsg(color, 'bgr8')
                image.header = depth_msg.header
                image_pub.publish(image)
                rospy.sleep(.065)

        frames(4.0, 20)  # PnP距离约1.5m，故意制造深度不一致。
        self.assertTrue(any(p.id == 2 for m in self.rough for p in m.platforms))
        self.assertFalse(any(m.platforms for m in self.confirmed), 'rejected depth must not authorize assignment')
        rough_point = [p.pose.pose.position for m in self.rough for p in m.platforms if p.id == 2][-1]
        self.assertAlmostEqual(rough_point.z, 1.5, delta=.1)
        frames(1.5, 20)
        self.assertTrue(any(p.id == 2 for m in self.confirmed for p in m.platforms))
        confirmed_sub.unregister()
        rough_sub.unregister()


if __name__ == '__main__':
    rospy.init_node('front_aruco_search_hint_test')
    rostest.rosrun('precision_landing', 'front_aruco_search_hint_test', FrontArucoSearchHintTest)
