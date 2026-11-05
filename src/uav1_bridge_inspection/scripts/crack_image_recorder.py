#!/usr/bin/env python3
"""保存含裂缝的同帧带框图、相机坐标和 FAST-LIO 地图坐标。"""
import json
import math
from pathlib import Path

import cv2
import message_filters
import rospy
import tf2_geometry_msgs  # 注册 PointStamped 的 TF 转换。
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from target_ros_msgs.msg import BoundingBoxes


class CrackImageRecorder:
    def __init__(self):
        root = Path(rospy.get_param('~record_root')).expanduser()
        mission_id = rospy.get_param(rospy.get_param('~mission_id_param')).strip()
        self.directory = root / mission_id / 'images' / 'annotated' / 'crack'
        self.directory.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.world_frame = rospy.get_param('~world_frame', 'UAV1/camera_init')
        self.camera_info = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.info_sub = rospy.Subscriber('camera_info', CameraInfo, self.info_cb, queue_size=1)
        self.image = message_filters.Subscriber('image', Image, queue_size=10,
                                                buff_size=33554432)
        self.objects = message_filters.Subscriber('objects', BoundingBoxes, queue_size=10)
        # YOLO 的检测结果 header 和带框图 header 均来自同一原始彩色帧。
        self.sync = message_filters.TimeSynchronizer([self.image, self.objects], 10)
        self.sync.registerCallback(self.save)
        rospy.loginfo('裂缝带框图保存目录: %s', self.directory)

    def info_cb(self, msg):
        self.camera_info = msg

    def locate(self, box, image, info):
        result = dict(class_name=box.Class, confidence=box.probability,
                      bbox=[box.xmin, box.ymin, box.xmax, box.ymax],
                      camera_position=None, world_position=None)
        # 现有 YOLO distance 单位为 mm，0 表示无有效同步深度。
        depth = float(box.distance) * .001
        if not math.isfinite(depth) or depth <= 0:
            result['status'] = 'no_valid_depth'
            return result
        if info is None or not info.header.frame_id:
            result['status'] = 'no_camera_info'
            return result
        fx, fy, cx, cy = info.K[0], info.K[4], info.K[2], info.K[5]
        if (info.width != image.width or info.height != image.height or
                not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0):
            result['status'] = 'invalid_camera_info'
            return result
        u, v = (box.xmin + box.xmax) / 2., (box.ymin + box.ymax) / 2.
        camera = PointStamped()
        camera.header.frame_id = info.header.frame_id
        camera.header.stamp = image.header.stamp
        camera.point.x, camera.point.y, camera.point.z = (u-cx)*depth/fx, (v-cy)*depth/fy, depth
        result['camera_position'] = dict(frame_id=info.header.frame_id, unit='m',
                                        x=camera.point.x, y=camera.point.y, z=depth)
        if image.header.stamp.is_zero():
            # TF 将零时间解释为最新位姿，不能用于代表未知曝光时刻。
            result['status'] = 'no_image_timestamp'
            return result
        try:
            world = self.tf_buffer.transform(camera, self.world_frame, rospy.Duration(.1))
            result['world_position'] = dict(frame_id=self.world_frame, unit='m',
                                           x=world.point.x, y=world.point.y, z=world.point.z)
            result['status'] = 'localized'
        except tf2_ros.TransformException as exc:
            result['status'] = 'tf_unavailable'
            rospy.logwarn_throttle(2., '裂缝坐标变换失败: %s', exc)
        return result

    def save(self, image, objects):
        boxes = [box for box in objects.bounding_boxes if box.Class == 'crack']
        if not boxes:
            return
        stamp = image.header.stamp
        path = self.directory / ('crack_%d_%09d_%d.png' %
                                 (stamp.secs, stamp.nsecs, image.header.seq))
        try:
            pixels = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            info = self.camera_info
            records = [self.locate(box, image, info) for box in boxes]
            lines = ['Map frame: %s | units: m | estimated box position' % self.world_frame]
            for index, record in enumerate(records, 1):
                for key, label in [('camera_position', 'Camera'), ('world_position', 'Map')]:
                    p = record[key]
                    text = ('X=%.3f Y=%.3f Z=%.3f' % (p['x'], p['y'], p['z'])
                            if p is not None else 'unavailable (%s)' % record['status'])
                    lines.append('crack #%d %s: %s' % (index, label, text))
            annotated = cv2.copyMakeBorder(pixels, 0, 24 * len(lines) + 12, 0, 0,
                                           cv2.BORDER_CONSTANT, value=(16, 16, 16))
            for index, line in enumerate(lines):
                cv2.putText(annotated, line, (10, pixels.shape[0] + 24 * (index + 1)),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv2.LINE_AA)
            if not cv2.imwrite(str(path), annotated):
                raise OSError('图像写入失败')
            record = dict(image=path.name, stamp=dict(secs=stamp.secs, nsecs=stamp.nsecs),
                          method='bbox_center_ray_with_yolo_depth', detections=records)
            path.with_suffix('.json').write_text(
                json.dumps(record, ensure_ascii=False, allow_nan=False, indent=2), encoding='utf-8')
        except (CvBridgeError, cv2.error, OSError) as exc:
            rospy.logerr('裂缝图保存失败 %s: %s', path, exc)


if __name__ == '__main__':
    rospy.init_node('crack_image_recorder')
    recorder = CrackImageRecorder()
    rospy.spin()
