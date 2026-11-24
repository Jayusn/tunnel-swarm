#!/usr/bin/env python3
"""Load a persistent FUEL snapshot; align a new boot near its original start.

Planner/mission use saved-map odometry. Commands are transformed back into the
unchanged controller's FAST-LIO frame. This is one-time local alignment, not SLAM
loop closure or arbitrary-position global localization.
"""
import copy
import json
import math
from pathlib import Path
import threading
import sys
import rospkg

import numpy as np
import rospy
from geometry_msgs.msg import Point
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, String, ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker
import colorsys

sys.path.insert(0, str(Path(rospkg.RosPack().get_path('rrt_minco_adapter')) / 'scripts'))
from saved_map_math import downsample, register, rotation


class SavedMap:
    def __init__(self):
        with open(Path(rospy.get_param('~map_file')).expanduser()) as stream:
            self.data = json.load(stream)
        if self.data['version'] != 1:
            raise ValueError('Unsupported saved map version')
        self.scene = self.data['scene']
        self.local_frame = self.scene['frame_id']
        self.frame = 'UAV0/saved_map'
        self.scene['frame_id'] = self.frame
        self.target = downsample(self.data['localization_points'])
        if len(self.target) < 100:
            raise ValueError('Saved map contains too few localization points')
        self.lock = threading.Lock()
        self.scans = []
        self.alignment = None
        self.state = None
        self.state_received = rospy.Time(0)
        self.odom_pub = rospy.Publisher('/UAV0/rrt/map_odom', Odometry, queue_size=1)
        self.command_pub = rospy.Publisher('/UAV0/planning/pos_cmd', PositionCommand, queue_size=1)
        self.scene_pub = rospy.Publisher('/UAV0/rrt/scene', String, queue_size=1, latch=True)
        self.voxel_pub = rospy.Publisher('/UAV0/rrt/voxel_map', Marker, queue_size=1, latch=True)
        self.scan_pub = rospy.Publisher('/UAV0/rrt/aligned_cloud', PointCloud2, queue_size=1)
        self.result_pub = rospy.Publisher('/UAV0/rrt/map_alignment', String, queue_size=1, latch=True)
        rospy.Subscriber('/UAV0/fast_lio/cloud_registered', PointCloud2, self.cloud, queue_size=1)
        rospy.Subscriber('/UAV0/fast_lio/Odom_high_freq', Odometry, self.odom, queue_size=1)
        rospy.Subscriber('/UAV0/rrt/map_pos_cmd', PositionCommand, self.command, queue_size=1)
        rospy.Subscriber('/UAV0/mavros/state', State, self.state_callback, queue_size=1)
        self.service = rospy.Service('/UAV0/rrt/freeze_map', Trigger, self.localize)
        self.publish_map()
        rospy.loginfo('Loaded saved map. Return near original start/heading, then call /UAV0/rrt/freeze_map to align this boot; no new mapping.')

    def state_callback(self, msg):
        self.state, self.state_received = msg, rospy.Time.now()

    def publish_map(self):
        marker = Marker()
        marker.header = Header(frame_id=self.frame, stamp=rospy.Time.now())
        marker.ns, marker.id, marker.type, marker.action = 'occupied_voxels', 0, Marker.CUBE_LIST, Marker.ADD
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = self.scene['resolution']
        marker.color.a = 1.
        centers = self.data['voxels']
        low, high = min(p[2] for p in centers), max(p[2] for p in centers)
        for x, y, z in centers:
            marker.points.append(Point(x=x, y=y, z=z))
            r, g, b = colorsys.hsv_to_rgb(.5 + (z-low)/max(high-low, 1.e-6)/3., 1., 1.)
            marker.colors.append(ColorRGBA(r=r, g=g, b=b, a=1.))
        self.voxel_pub.publish(marker)

    def cloud(self, msg):
        if self.alignment is not None and not self.scan_pub.get_num_connections():
            return
        if msg.header.frame_id != self.local_frame:
            rospy.logwarn_throttle(5., 'Live cloud frame differs from saved source frame')
            return
        points = downsample(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)))
        with self.lock:
            if self.alignment is None:
                self.scans.append((rospy.Time.now(), points))
                self.scans = self.scans[-20:]
            alignment = self.alignment
        if alignment is not None:
            yaw, shift = alignment
            self.scan_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=msg.header.stamp, frame_id=self.frame), points @ rotation(yaw).T + shift))

    def localize(self, _request):
        with self.lock:
            if self.alignment is not None:
                return TriggerResponse(False, 'Already aligned; restart saved-map entry to align again on ground')
            if self.state is None or self.state.armed or (rospy.Time.now()-self.state_received).to_sec() > 2.:
                return TriggerResponse(False, 'Require fresh disarmed MAVROS state for boot alignment')
            if not self.scans or (rospy.Time.now()-self.scans[-1][0]).to_sec() > 1.:
                return TriggerResponse(False, 'No fresh scan received')
            source = np.concatenate([p for stamp, p in self.scans if (rospy.Time.now()-stamp).to_sec() < 3.])
            try:
                initial = rospy.get_param('~initial_pose', [0., 0., 0., 0.])
                # Saved FUEL map is height-limited; do not penalize newer scans above that map.
                low, high = self.target.min(axis=0)-.5, self.target.max(axis=0)+.5
                guess = source @ rotation(initial[3]).T + initial[:3]
                source = source[((guess >= low) & (guess <= high)).all(axis=1)]
                yaw, shift, score = register(source, self.target, initial)
                if score['inlier_ratio'] < .6 or score['rmse_m'] > .10:
                    raise ValueError('匹配质量不足: ' + json.dumps(score))
                if self.state.armed:
                    raise ValueError('Armed during alignment; abort')
                result = dict(yaw_rad=yaw, translation=shift.tolist(), **score)
                self.alignment = (yaw, shift)
                self.scans.clear()
                self.result_pub.publish(String(data=json.dumps(result)))
                self.scene_pub.publish(String(data=json.dumps(self.scene, allow_nan=False)))
                return TriggerResponse(True, 'Saved map aligned: ' + json.dumps(result))
            except (ValueError, IndexError) as exc:
                return TriggerResponse(False, str(exc))

    def odom(self, msg):
        alignment = self.alignment
        if alignment is None or msg.header.frame_id != self.local_frame:
            return
        yaw, shift = alignment
        r = rotation(yaw)
        msg = copy.deepcopy(msg)
        msg.header.frame_id = self.frame
        p = msg.pose.pose.position
        p.x, p.y, p.z = r @ [p.x, p.y, p.z] + shift
        q = msg.pose.pose.orientation
        c, s = math.cos(yaw/2), math.sin(yaw/2)
        q.x, q.y, q.z, q.w = c*q.x-s*q.y, c*q.y+s*q.x, c*q.z+s*q.w, c*q.w-s*q.z
        # FAST-LIO in this workspace explicitly publishes world-frame linear velocity.
        v = msg.twist.twist.linear
        v.x, v.y, v.z = r @ [v.x, v.y, v.z]
        block = np.zeros((6, 6))
        block[:3, :3] = block[3:, 3:] = r
        msg.pose.covariance = (block @ np.array(msg.pose.covariance).reshape(6, 6) @ block.T).ravel().tolist()
        velocity_block = np.eye(6)
        velocity_block[:3, :3] = r
        msg.twist.covariance = (velocity_block @ np.array(msg.twist.covariance).reshape(6, 6) @ velocity_block.T).ravel().tolist()
        self.odom_pub.publish(msg)

    def command(self, msg):
        alignment = self.alignment
        if alignment is None or msg.header.frame_id != self.frame:
            return
        yaw, shift = alignment
        r = rotation(yaw).T
        msg = copy.deepcopy(msg)
        msg.header.frame_id = self.local_frame
        p = msg.position
        p.x, p.y, p.z = r @ (np.array([p.x, p.y, p.z])-shift)
        for v in (msg.velocity, msg.acceleration, msg.jerk):
            v.x, v.y, v.z = r @ [v.x, v.y, v.z]
        msg.yaw = math.atan2(math.sin(msg.yaw-yaw), math.cos(msg.yaw-yaw))
        self.command_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('rrt_saved_map')
    SavedMap()
    rospy.spin()
