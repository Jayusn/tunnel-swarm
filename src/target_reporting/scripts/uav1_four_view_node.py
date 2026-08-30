#!/usr/bin/env python3
"""Combine five UAV1 detector images into one display image."""

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class FourViewNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = __import__("threading").Lock()
        self.frames = [None, None, None, None, None]
        self.output_width = int(rospy.get_param("~output_width", 1280))
        self.output_height = int(rospy.get_param("~output_height", 720))
        topics = [
            "/UAV1/color_tag_detector/debug_image",
            "/UAV1/vision/qr_debug_image",
            "/UAV1/landing/search/debug_image",
            "/UAV1/thermal/debug_image",
            "/UAV1/thermal/d435_debug_image",
        ]
        self.publisher = rospy.Publisher(
            "/UAV1/detection/four_view", Image, queue_size=1
        )
        for index, topic in enumerate(topics):
            rospy.Subscriber(topic, Image, self.image_callback, index, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / 15.0), self.publish)

    def image_callback(self, msg, index):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Cannot convert four-view image: %s", exc)
            return
        with self.lock:
            self.frames[index] = image.copy()

    @staticmethod
    def fit_image(image, width, height):
        canvas = __import__("numpy").zeros((height, width, 3), dtype=__import__("numpy").uint8)
        if image is None:
            return canvas
        image_height, image_width = image.shape[:2]
        scale = min(float(width) / image_width, float(height) / image_height)
        resized = cv2.resize(
            image,
            (max(1, int(round(image_width * scale))), max(1, int(round(image_height * scale)))),
            interpolation=cv2.INTER_AREA,
        )
        y = (height - resized.shape[0]) // 2
        x = (width - resized.shape[1]) // 2
        canvas[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
        return canvas

    def publish(self, _event):
        with self.lock:
            frames = list(self.frames)
        top_height = self.output_height // 2
        bottom_height = self.output_height - top_height
        mosaic = __import__("numpy").zeros(
            (self.output_height, self.output_width, 3), dtype=__import__("numpy").uint8
        )
        for index, frame in enumerate(frames[:3]):
            x0 = index * self.output_width // 3
            x1 = (index + 1) * self.output_width // 3
            tile = self.fit_image(frame, x1 - x0, top_height)
            cv2.rectangle(tile, (0, 0), (tile.shape[1] - 1, tile.shape[0] - 1), (255, 255, 255), 1)
            mosaic[0:top_height, x0:x1] = tile
        for index, frame in enumerate(frames[3:]):
            x0 = index * self.output_width // 2
            x1 = (index + 1) * self.output_width // 2
            tile = self.fit_image(frame, x1 - x0, bottom_height)
            cv2.rectangle(tile, (0, 0), (tile.shape[1] - 1, tile.shape[0] - 1), (255, 255, 255), 1)
            mosaic[top_height:self.output_height, x0:x1] = tile
        message = self.bridge.cv2_to_imgmsg(mosaic, encoding="bgr8")
        message.header.stamp = rospy.Time.now()
        self.publisher.publish(message)


if __name__ == "__main__":
    rospy.init_node("uav1_four_view")
    FourViewNode()
    rospy.spin()
