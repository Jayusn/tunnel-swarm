#!/usr/bin/env python3
"""隔离 ROS master，验证带框图与检测帧严格配对后落盘。"""
import sys
import json
import math
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
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
            np.testing.assert_array_equal(cv2.imread(str(files()[0]))[:120], annotated)
            self.assertTrue(files()[0].name.startswith('crack_21_'))
            # 相反到达顺序及连续检测帧也都能正常保存。
            images.publish(frame(22, annotated))
            objects.publish(detections(22))
            self.wait(lambda: len(files()) == 2 and cv2.imread(str(files()[1])) is not None,
                      'second annotated image saved')
            np.testing.assert_array_equal(cv2.imread(str(files()[1]))[:120], annotated)
            self.wait(lambda: files()[1].with_suffix('.json').exists(), 'coordinate record')
            record = json.loads(files()[1].with_suffix('.json').read_text())['detections'][0]
            self.assertEqual(record['status'], 'no_valid_depth')
            self.assertIsNone(record['camera_position'])
            self.assertIsNone(record['world_position'])
            objects.publish(detections(23, False))
            images.publish(frame(23, plain))
            time.sleep(.2)
            self.assertEqual(len(files()), 2)

    def test_camera_and_world_coordinates_at_image_time(self):
        with tempfile.TemporaryDirectory(prefix='bridge_crack_xyz_test_') as root:
            rospy.set_param(self.prefix + '/recorder/record_root', root)
            rospy.set_param(self.prefix + '/recorder/world_frame', 'test_map')
            self.launch('uav1_bridge_inspection', 'crack_image_recorder.py', 'recorder')
            images = self.publisher('image', Image)
            objects = self.publisher('objects', BoundingBoxes)
            infos = self.publisher('camera_info', CameraInfo, latch=True)
            self.wait(lambda: all(p.get_num_connections() for p in (images, objects, infos)),
                      'coordinate recorder subscriptions')
            info = CameraInfo(width=160, height=120)
            info.header.frame_id = 'test_optical'
            info.K = [100., 0., 60., 0., 120., 30., 0., 0., 1.]
            infos.publish(info)
            broadcaster = tf2_ros.TransformBroadcaster()
            self.wait(lambda: broadcaster.pub_tf.get_num_connections(), 'TF subscriber')
            transforms = []
            for stamp, scale in [(30, 1), (31, 10)]:
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = 'test_map', 'test_optical'
                tf.header.stamp = rospy.Time(stamp)
                tf.transform.translation.x = 10. * scale
                tf.transform.translation.y = 20. * scale
                tf.transform.translation.z = 30. * scale
                tf.transform.rotation.z = math.sin(math.pi / 4)
                tf.transform.rotation.w = math.cos(math.pi / 4)
                transforms.append(tf)
            broadcaster.sendTransform(transforms)
            time.sleep(.2)
            image = CvBridge().cv2_to_imgmsg(np.zeros((120, 160, 3), np.uint8), 'bgr8')
            image.header.stamp = rospy.Time(30)
            boxes = BoundingBoxes()
            boxes.header = image.header
            boxes.bounding_boxes = [BoundingBox(Class='crack', probability=.9, distance=2000.,
                                               xmin=60, xmax=100, ymin=40, ymax=80),
                                    BoundingBox(Class='crack', probability=.8, distance=0.)]
            images.publish(image)
            objects.publish(boxes)

            def saved(stamp):
                paths = list(Path(root).rglob('crack_%d_*.json' % stamp))
                return paths[0] if paths and paths[0].read_text().endswith('}') else None

            self.wait(lambda: saved(30), 'localized record')
            records = json.loads(saved(30).read_text())['detections']
            self.assertEqual(records[0]['status'], 'localized')
            camera, world = records[0]['camera_position'], records[0]['world_position']
            for key, value in zip(('x', 'y', 'z'), (.4, .5, 2.)):
                self.assertAlmostEqual(camera[key], value)
            # 必须使用 t=30 的旋转和平移，不能使用 t=31 的最新位姿。
            for key, value in zip(('x', 'y', 'z'), (9.5, 20.4, 32.)):
                self.assertAlmostEqual(world[key], value)
            self.assertEqual(world['frame_id'], 'test_map')
            self.assertEqual(world['unit'], 'm')
            self.assertEqual(records[1]['status'], 'no_valid_depth')
            self.assertIsNone(records[1]['world_position'])
            annotated = cv2.imread(str(saved(30).with_suffix('.png')))
            self.assertGreater(annotated.shape[0], 120)
            self.assertGreater(np.max(annotated[120:]), 16)
            # TF 时间范围之外仍存图，保留相机坐标，不伪造地图坐标。
            image.header.stamp = rospy.Time(32)
            boxes.header = image.header
            images.publish(image)
            objects.publish(boxes)
            self.wait(lambda: saved(32), 'missing TF record')
            missing = json.loads(saved(32).read_text())['detections'][0]
            self.assertEqual(missing['status'], 'tf_unavailable')
            self.assertIsNotNone(missing['camera_position'])
            self.assertIsNone(missing['world_position'])
            image.header.stamp = rospy.Time(0)
            boxes.header = image.header
            images.publish(image)
            objects.publish(boxes)
            self.wait(lambda: saved(0), 'zero timestamp record')
            missing = json.loads(saved(0).read_text())['detections'][0]
            self.assertEqual(missing['status'], 'no_image_timestamp')
            self.assertIsNone(missing['world_position'])


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite([
        CrackRecordingTest('test_save_matching_annotated_frames'),
        CrackRecordingTest('test_camera_and_world_coordinates_at_image_time')]))
    sys.exit(not result.wasSuccessful())
