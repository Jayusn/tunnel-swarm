#!/usr/bin/env python3
"""按固定时间间隔将收到的彩色原图保存到同一个采集目录。"""
from datetime import datetime
import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image


class ColorImageCapture:
    def __init__(self):
        self.interval = float(rospy.get_param('~interval_sec', 1.0))
        if not math.isfinite(self.interval) or self.interval <= 0:
            raise ValueError('interval_sec 必须是大于 0 的有限数值')
        root = Path(rospy.get_param('~output_root')).expanduser()
        self.directory = root / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True, exist_ok=False)
        self.bridge = CvBridge()
        self.count = 0
        self.last_saved = None
        rospy.loginfo('采集目录：%s；每 %.2f 秒保存一张 PNG；等待彩色图像。',
                      self.directory, self.interval)
        self.sub = rospy.Subscriber('image', Image, self.save, queue_size=1,
                                    buff_size=16 * 1024 * 1024)

    def save(self, msg):
        now = time.monotonic()
        if self.last_saved is not None and now - self.last_saved < self.interval:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            rospy.logerr_throttle(5.0, '彩色图像转换失败：%s', exc)
            return
        filename = self.directory / ('%06d_%d.png' %
                                     (self.count + 1, msg.header.stamp.to_nsec()))
        try:
            if not cv2.imwrite(str(filename), frame):
                raise OSError('PNG 写入失败')
        except (cv2.error, OSError) as exc:
            rospy.logerr('保存失败 %s：%s', filename, exc)
            rospy.signal_shutdown('无法保存采集图片')
            return
        self.count += 1
        self.last_saved = now
        rospy.loginfo('已保存 %d 张：%s', self.count, filename.name)


if __name__ == '__main__':
    rospy.init_node('crack_photo_capture')
    capture = ColorImageCapture()
    rospy.spin()
