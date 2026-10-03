#!/usr/bin/env python3
"""Exercise FAPP against synthetic registered clouds on an isolated ROS master."""
import math
import os
import signal
import subprocess
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
from obj_state_msgs.msg import ObjectsStates


def main():
    master = subprocess.Popen(['roscore', '-p', '11431'], stdout=subprocess.DEVNULL,
                              stderr=subprocess.STDOUT, start_new_session=True)
    launch = None
    rviz_processes = []
    try:
        time.sleep(2)
        rospy.init_node('fapp_smoke', anonymous=True)
        launch_log = open('/tmp/match_fapp_smoke.log', 'w')
        launch = subprocess.Popen(['roslaunch', 'mot_mapping', 'uav1_detection.launch',
                                   'input_rate:=20'], stdout=launch_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        received = {'cloud': [], 'boxes': [], 'tracks': [], 'states': []}
        subscribers = [
            rospy.Subscriber('/UAV1/fapp/' + topic, msg_type,
                             lambda msg, key=key: received[key].append(msg))
            for topic, msg_type, key in [
                ('dynamic_cloud', PointCloud2, 'cloud'),
                ('dynamic_object_markers', MarkerArray, 'boxes'),
                ('dynamic_track_markers', MarkerArray, 'tracks'),
                ('states', ObjectsStates, 'states')]]
        cloud_pub = rospy.Publisher('/UAV1/fast_lio/cloud_registered', PointCloud2, queue_size=10)
        odom_pub = rospy.Publisher('/UAV1/fast_lio/Odom_high_freq', Odometry, queue_size=10)
        time.sleep(3)
        if os.environ.get('FAPP_SMOKE_RVIZ') == '1':
            workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
            for profile in ['uav1_lite', 'exp']:
                config = os.path.join(workspace, 'src/Diff-Planner/src/diff_planner/',
                                      'plan_manage/launch/include/', profile + '.rviz')
                with open('/tmp/match_fapp_rviz_' + profile + '.log', 'w') as rviz_log:
                    rviz_processes.append(subprocess.Popen(
                        ['rviz', '-d', config, '__name:=fapp_smoke_rviz_' + profile],
                        stdout=rviz_log, stderr=subprocess.STDOUT, start_new_session=True))
            time.sleep(5)
            assert all(process.poll() is None for process in rviz_processes), 'RViz exited'

        def publish(center):
            stamp = rospy.Time.now()
            header = Header(stamp=stamp, frame_id='UAV1/camera_init')
            odom = Odometry(header=header)
            odom.pose.pose.orientation.w = 1
            points = [] if center is None else [
                (center + x * .055, y * .055, 1 + z * .055)
                for x in range(4) for y in range(4) for z in range(4)]
            odom_pub.publish(odom)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
            time.sleep(.25)
            return stamp

        for center in [2, 2, 2, 2.3, 2.45, 2.6, 2.75]:
            publish(center)
        assert launch.poll() is None, 'FAPP node exited'
        print('Received:', {key: len(value) for key, value in received.items()})
        assert any(msg.width > 0 for msg in received['cloud']), 'No dynamic points'
        boxes = [marker for msg in received['boxes'] for marker in msg.markers
                 if marker.action == marker.ADD]
        assert boxes, 'No detection boxes'
        assert all(marker.header.frame_id == 'UAV1/camera_init' and
                   not marker.header.stamp.is_zero() for marker in boxes)
        assert any(msg.states for msg in received['states']), 'No tracked state'
        assert any(any(m.action == m.ADD for m in msg.markers)
                   for msg in received['tracks']), 'No tracking direction'
        assert all(math.isfinite(value) for msg in received['states']
                   for state in msg.states for value in
                   (state.velocity.x, state.velocity.y, state.velocity.z))
        for _ in range(6):
            last_stamp = publish(None)
        assert received['boxes'][-1].markers[0].action == 3, 'Missing DELETEALL'
        assert len(received['boxes'][-1].markers) == 1, 'Old detection box remains'
        assert received['cloud'][-1].header.stamp == last_stamp, 'Cloud timestamp lost'
        if rviz_processes:
            _, _, graph = rospy.get_master().getSystemState()
            subscribers_by_topic = dict(graph[1])
            for topic in ['dynamic_cloud', 'dynamic_object_markers', 'dynamic_track_markers']:
                nodes = subscribers_by_topic.get('/UAV1/fapp/' + topic, [])
                assert all('/fapp_smoke_rviz_' + profile in nodes
                           for profile in ['uav1_lite', 'exp']), 'RViz display not subscribed: ' + topic
            print('PASS: both RViz profiles subscribe to FAPP clouds and markers')
        print('PASS: dynamic clouds, boxes, tracking, frame/timestamps and box clearing')
    finally:
        for process in rviz_processes + [launch, master]:
            if process is not None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)


if __name__ == '__main__':
    main()
