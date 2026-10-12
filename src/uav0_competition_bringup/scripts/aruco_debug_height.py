#!/usr/bin/env python3
"""在 ArUco 调试画面下方显示当前 FAST-LIO XYZ，不发布控制指令。"""

import math

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


def height_label(odometry, now):
    if odometry is None:
        return "LIO XYZ: waiting for odometry"
    age = (now - odometry.header.stamp).to_sec()
    position = odometry.pose.pose.position
    xyz = (position.x, position.y, position.z)
    if odometry.header.stamp.is_zero() or not all(math.isfinite(v) for v in xyz):
        return "LIO XYZ: invalid odometry"
    if age < -0.05 or age > 0.30:
        return "LIO XYZ: odometry stale"
    return "LIO XYZ (m): X={:+.2f} Y={:+.2f} Z={:+.2f}".format(*xyz)


class HeightOverlay:
    def __init__(self):
        self.bridge = CvBridge()
        self.odometry = None
        self.publishers = {}
        self.subscribers = [rospy.Subscriber(
            rospy.get_param("~odometry_topic", "/UAV0/fast_lio/Odom_high_freq"),
            Odometry, self.on_odometry, queue_size=1)]
        for view in ("front", "down"):
            prefix = "/UAV0/aruco_observer/" + view
            self.publishers[view] = rospy.Publisher(
                prefix + "/height_image", Image, queue_size=1)
            self.subscribers.append(rospy.Subscriber(
                prefix + "/debug_image", Image, self.on_image,
                callback_args=view, queue_size=1, buff_size=2**24))

    def on_odometry(self, message):
        self.odometry = message

    def on_image(self, message, view):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
            # 增加底栏，保留完整的检测框、ID 和原有状态文字。
            image = cv2.copyMakeBorder(frame, 0, 36, 0, 0, cv2.BORDER_CONSTANT)
            cv2.putText(image, height_label(self.odometry, rospy.Time.now()),
                        (10, image.shape[0] - 11), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 255), 1, cv2.LINE_AA)
            output = self.bridge.cv2_to_imgmsg(image, "bgr8")
            output.header = message.header
            self.publishers[view].publish(output)
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "ArUco height image: %s", error)


if __name__ == "__main__":
    rospy.init_node("aruco_debug_height")
    overlay = HeightOverlay()
    rospy.spin()
