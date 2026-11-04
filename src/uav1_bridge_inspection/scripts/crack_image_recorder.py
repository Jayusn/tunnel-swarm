#!/usr/bin/env python3
"""保存含裂缝的同帧带框图，不使用无时间戳的检测布尔量配对。"""
from datetime import datetime
from pathlib import Path

import cv2
import message_filters
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from target_ros_msgs.msg import BoundingBoxes


class CrackImageRecorder:
    def __init__(self):
        root = Path(rospy.get_param('~record_root')).expanduser()
        self.directory = root / datetime.now().strftime('%Y%m%d/flight_%H%M%S_%f')
        self.directory.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.image = message_filters.Subscriber('image', Image, queue_size=10,
                                                buff_size=33554432)
        self.objects = message_filters.Subscriber('objects', BoundingBoxes, queue_size=10)
        # YOLO 的检测结果 header 和带框图 header 均来自同一原始彩色帧。
        self.sync = message_filters.TimeSynchronizer([self.image, self.objects], 10)
        self.sync.registerCallback(self.save)
        rospy.loginfo('裂缝带框图保存目录: %s', self.directory)

    def save(self, image, objects):
        if not any(box.Class == 'crack' for box in objects.bounding_boxes):
            return
        stamp = image.header.stamp
        path = self.directory / ('crack_%d_%09d_%d.png' %
                                 (stamp.secs, stamp.nsecs, image.header.seq))
        try:
            pixels = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            if not cv2.imwrite(str(path), pixels):
                raise OSError('图像写入失败')
        except (CvBridgeError, cv2.error, OSError) as exc:
            rospy.logerr('裂缝图保存失败 %s: %s', path, exc)


if __name__ == '__main__':
    rospy.init_node('crack_image_recorder')
    recorder = CrackImageRecorder()
    rospy.spin()
