#!/usr/bin/env python3
"""隔离 ROS master，验证带框图与检测帧严格配对后落盘。"""
import sys
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from target_ros_msgs.msg import BoundingBox, BoundingBoxes

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'uav1_inplace_landing_test/test'))
from test_rc_land_priority_ros import RcLandPriorityTest


class CrackRecordingTest(RcLandPriorityTest):
    def test_save_matching_annotated_frames(self):
        with tempfile.TemporaryDirectory(prefix='bridge_crack_test_') as root:
            rospy.set_param(self.prefix + '/recorder/record_root', root)
            self.launch('uav1_bridge_inspection', 'crack_image_recorder.py', 'recorder')
            images = self.publisher('image', Image)
            objects = self.publisher('objects', BoundingBoxes)
            self.wait(lambda: images.get_num_connections() and objects.get_num_connections(),
                      'recorder subscriptions')
            bridge = CvBridge()
            plain = np.zeros((120, 160, 3), dtype=np.uint8)
            annotated = plain.copy()
            cv2.rectangle(annotated, (20, 30), (100, 80), (0, 220, 255), 3)
            cv2.putText(annotated, 'crack 90%', (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 220, 255), 1)

            def frame(stamp, pixels):
                msg = bridge.cv2_to_imgmsg(pixels, 'bgr8')
                msg.header.stamp = rospy.Time(stamp)
                return msg

            def detections(stamp, found=True):
                msg = BoundingBoxes()
                msg.header.stamp = rospy.Time(stamp)
                msg.image_header = msg.header
                if found:
                    msg.bounding_boxes = [BoundingBox(Class='crack', probability=.9)]
                return msg

            files = lambda: sorted(Path(root).rglob('*.png'))
            # 无目标帧和时间不匹配的图像均不得保存。
            images.publish(frame(10, plain))
            objects.publish(detections(10, False))
            images.publish(frame(20, plain))
            objects.publish(detections(21))
            time.sleep(.3)
            self.assertEqual(files(), [])
            # 检测先到，之后必须保存 stamp=21 的带框图，而不是旧的无框图。
            images.publish(frame(21, annotated))
            self.wait(lambda: len(files()) == 1 and cv2.imread(str(files()[0])) is not None,
                      'first annotated image saved')
            np.testing.assert_array_equal(cv2.imread(str(files()[0])), annotated)
            self.assertTrue(files()[0].name.startswith('crack_21_'))
            # 相反到达顺序及连续检测帧也都能正常保存。
            images.publish(frame(22, annotated))
            objects.publish(detections(22))
            self.wait(lambda: len(files()) == 2 and cv2.imread(str(files()[1])) is not None,
                      'second annotated image saved')
            np.testing.assert_array_equal(cv2.imread(str(files()[1])), annotated)
            objects.publish(detections(23, False))
            images.publish(frame(23, plain))
            time.sleep(.2)
            self.assertEqual(len(files()), 2)


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite([
        CrackRecordingTest('test_save_matching_annotated_frames')]))
    sys.exit(not result.wasSuccessful())
