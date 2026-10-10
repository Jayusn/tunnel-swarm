#!/usr/bin/env python3
"""Isolated ROS replay adapter. Never connected to a production launch."""
import json
import hashlib
from pathlib import Path
import time

import message_filters
import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from obj_state_msgs.msg import ObjectsStates, State
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from event_core import EventDetector, LATERAL, DEPTH, REGION_RADIUS


def xyz(message):
    fields = {f.name: f for f in message.fields}
    assert all(fields[name].datatype == 7 and fields[name].count == 1 for name in ('x', 'y', 'z'))
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=[endian + 'f4'] * 3,
                         offsets=[fields[n].offset for n in ('x', 'y', 'z')], itemsize=message.point_step))
    structured = np.ndarray((message.height, message.width), dtype=dtype,
                            buffer=message.data, strides=(message.row_step, message.point_step))
    return np.column_stack([structured[name].ravel() for name in ('x', 'y', 'z')]).astype(float)


def point(values):
    return Point(*map(float, values))


class ReplayNode:
    def __init__(self):
        def param(key, default):
            return rospy.get_param('~' + key, default)
        self.frame = param('ros/frame_id', 'UAV1/camera_init')
        self.offset = np.array(rospy.get_param('/UAV1/mapping/extrinsic_T', [-.011, -.02329, .04412]), dtype=float)
        self.max_above_uav = param('detection/max_above_uav', 1.5)
        self.detector = EventDetector(
            capture=param('background_reference/capture_duration', 5.),
            min_cluster=param('dbscan/min_cluster', 6), max_cluster=param('dbscan/max_cluster', 800),
            confirmation_frames=param('detection/motion_confirmation_frames', 3),
            displacement=param('detection/motion_displacement', .06),
            speed=param('detection/min_motion_speed', .14),
            novel_fraction=param('detection/min_motion_novel_fraction', .26),
            tracked_novel_fraction=param('detection/tracked_motion_novel_fraction', .20))
        self.cloud_pub = rospy.Publisher('/UAV1/fapp/dynamic_cloud', PointCloud2, queue_size=10)
        self.state_pub = rospy.Publisher('/UAV1/fapp/states', ObjectsStates, queue_size=10)
        self.box_pub = rospy.Publisher('/UAV1/fapp/dynamic_object_markers', MarkerArray, queue_size=10)
        self.arrow_pub = rospy.Publisher('/UAV1/fapp/dynamic_track_markers', MarkerArray, queue_size=10)
        self.cloud_sub = message_filters.Subscriber(param('ros/lidar_topic', '/UAV1/fast_lio/cloud_registered'), PointCloud2, queue_size=10)
        self.odom_sub = message_filters.Subscriber(param('ros/odom_topic', '/UAV1/fast_lio/Odom_high_freq'), Odometry, queue_size=25)
        # This isolated adapter pairs real messages within 50 ms. It does not
        # modify the production synchronizer or repair abnormal source poses.
        self.sync = message_filters.ApproximateTimeSynchronizer([self.cloud_sub, self.odom_sub], queue_size=100, slop=.05)
        self.sync.registerCallback(self.callback)
        rospy.loginfo('POINT_EVENT_CONFIG ' + json.dumps(dict(
            implementation='frame-input point-occlusion prototype; not full M-detector',
            sensor_offset=self.offset.tolist(), ray_lateral=LATERAL, ray_depth=DEPTH,
            region_radius=REGION_RADIUS, actual_confirmation_frames=3,
            no_point_timestamps=True, angular_index='unit-direction KD tree',
            clustering='connected region growth of actual non-static points',
            synchronization_slop_seconds=.05, frozen_map=False,
            map_consistency='last two non-event real frames',
            core_sha256=hashlib.sha256(Path(__file__).with_name('event_core.py').read_bytes()).hexdigest(),
            candidate_roi='world XY distance from current body position < 5m; no point Z clipping',
            actual_parameters=dict(capture=self.detector.capture,
                min_cluster=self.detector.min_cluster,max_cluster=self.detector.max_cluster,
                displacement=self.detector.displacement,speed=self.detector.speed,
                novel_fraction=self.detector.novel_fraction,tracked_novel_fraction=self.detector.tracked_novel_fraction,
                max_above_uav=self.max_above_uav)), ensure_ascii=False))

    @staticmethod
    def marker(header, identifier, kind):
        marker = Marker(header=header, ns='point_event', id=identifier, type=kind, action=Marker.ADD)
        marker.pose.orientation.w = 1.
        marker.lifetime = rospy.Duration(.3)
        marker.color.a = 1.
        return marker

    def callback(self, cloud, odom):
        started = time.perf_counter()
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        position = np.array([p.x, p.y, p.z])
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        origin = position + rotation @ self.offset
        points = xyz(cloud)
        eligible = np.linalg.norm(points[:, :2] - position[:2], axis=1) < 5.
        height_limit = position[2] + self.max_above_uav if self.max_above_uav >= 0 else None
        result = self.detector.step(cloud.header.stamp.to_sec(), points, rotation, origin, eligible, height_limit)
        header = Header(stamp=cloud.header.stamp, frame_id=self.frame)
        states, boxes, arrows = ObjectsStates(header=header), MarkerArray(), MarkerArray()
        clear_box = Marker(header=header, action=Marker.DELETEALL)
        clear_arrow = Marker(header=header, action=Marker.DELETEALL)
        boxes.markers.append(clear_box); arrows.markers.append(clear_arrow)
        dynamic_indices = []
        for index, value in enumerate(result['published']):
            state = State(header=header)
            for name in ('position', 'size', 'velocity'):
                vector = value['center'] if name == 'position' else value[name]
                target = getattr(state, name)
                target.x, target.y, target.z = map(float, vector)
            states.states.append(state)
            center, size = value['center'], value['size']
            low, high = center - size / 2, center + size / 2
            corners = [[low[0], low[1], low[2]], [low[0], low[1], high[2]], [low[0], high[1], high[2]], [low[0], high[1], low[2]],
                       [high[0], low[1], low[2]], [high[0], low[1], high[2]], [high[0], high[1], high[2]], [high[0], high[1], low[2]]]
            box = self.marker(header, index, Marker.LINE_LIST)
            box.scale.x = .03
            box.color.r, box.color.g = 1., .5
            for a, b in [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]:
                box.points += [point(corners[a]), point(corners[b])]
            boxes.markers.append(box)
            arrow = self.marker(header, 100 + index, Marker.ARROW)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = .05, .1, .2
            arrow.color.g = 1.
            velocity = value['velocity']
            direction = velocity / np.linalg.norm(velocity) if np.linalg.norm(velocity) > .01 else np.zeros(3)
            arrow.points = [point(center), point(center + direction)]
            arrows.markers.append(arrow)
            dynamic_indices.extend(value['indices'].tolist())
        self.state_pub.publish(states)
        self.box_pub.publish(boxes)
        self.arrow_pub.publish(arrows)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, result['points'][dynamic_indices]))
        ns = cloud.header.stamp.to_nsec()
        # Frozen scorers can read current measured positions and reasons;
        # no ground truth enters this node.
        for decision in result['decisions']:
            rospy.loginfo('POINT_EVENT_CANDIDATE %d %s', ns, json.dumps(decision, separators=(',', ':')))
        kinds = result['event_types']
        rospy.loginfo('POINT_EVENT_FRAME %d %s', ns, json.dumps(dict(
            capture=result['capturing'], chain=result['point_chain_valid'], input=len(result['points']),
            candidate_input=int(np.sum(eligible & np.isfinite(points).all(axis=1))),
            events=int(np.sum(kinds != 0)), static=int(np.sum(result['static'])),
            crossing=int(np.sum((kinds & 1) != 0)), toward=int(np.sum((kinds & 2) != 0)), away=int(np.sum((kinds & 4) != 0)),
            candidates=len(result['candidates']), output=len(result['published']), odom_dt_ms=1000*(odom.header.stamp-cloud.header.stamp).to_sec()), separators=(',', ':')))
        # Same callback marker syntax as the production evaluation harness.
        rospy.loginfo('FAPP_CALLBACK_TIME %d %.9f', ns, 1000 * (time.perf_counter() - started))


if __name__ == '__main__':
    rospy.init_node('point_event_prototype')
    ReplayNode()
    rospy.spin()
