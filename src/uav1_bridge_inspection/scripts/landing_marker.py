#!/usr/bin/env python3
"""将精降已识别的 ArUco 位置画成平台块；只发布 RViz 标记。"""
import message_filters
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class LandingMarker:
    def __init__(self):
        config = rospy.get_param('~landing_config_ns').rstrip('/')
        # 读取精降实际使用的外参和中心补偿，避免另维护一套标定。
        self.rotation = np.asarray(rospy.get_param(config + '/camera_to_body/rotation'))
        self.translation = np.asarray(rospy.get_param(config + '/camera_to_body/translation_m'))
        self.bias = np.asarray(rospy.get_param(config + '/center_alignment/bias_xy_m'))
        self.publisher = rospy.Publisher('marker', Marker, queue_size=1, latch=True)
        clear = Marker(action=Marker.DELETEALL)
        self.publisher.publish(clear)
        self.target = message_filters.Subscriber('target_pose', PoseStamped, queue_size=10)
        self.odom = message_filters.Subscriber('odometry', Odometry, queue_size=100)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.target, self.odom], queue_size=100, slop=0.05)
        self.sync.registerCallback(self.publish)

    def publish(self, target, odom):
        p = target.pose.position
        offset = self.rotation.dot([p.x, p.y, p.z]) + self.translation
        offset[:2] -= self.bias
        q = odom.pose.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w])
        norm = np.linalg.norm(quaternion)
        if not odom.header.frame_id or not np.isfinite(norm) or norm < 1e-9:
            return
        quaternion /= norm
        # 四元数旋转：与精降将相机观测投影到 FAST-LIO 世界系的方法一致。
        v, w = quaternion[:3], quaternion[3]
        rotated = offset + 2.0 * np.cross(v, np.cross(v, offset) + w * offset)
        origin = odom.pose.pose.position
        position = np.array([origin.x, origin.y, origin.z]) + rotated
        if not np.all(np.isfinite(position)):
            return
        marker = Marker()
        marker.header.frame_id = odom.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'bridge_landing_platform'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.pose.orientation.w = 1.0
        # 沿用多机降落平台的青色扁方块，尺寸仅用于显示，不代表码的边长。
        marker.scale.x = marker.scale.y = 0.60
        marker.scale.z = 0.08
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .05, .95, .95, .95
        # 丢码后保留最后一次观测位置，与多机降落块一致。
        self.publisher.publish(marker)


if __name__ == '__main__':
    rospy.init_node('bridge_landing_marker')
    node = LandingMarker()
    rospy.spin()
