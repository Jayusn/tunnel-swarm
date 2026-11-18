#!/usr/bin/env python3
"""Run with match_ws sourced; creates and tears down an isolated ROS master."""
import json
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xmlrpc.client


def wait_for(predicate, timeout=15):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('Timed out waiting for ROS test condition')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cloud', action='store_true')
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ['ROS_IP'] = '127.0.0.1'
    directory = Path(tempfile.mkdtemp(prefix='rrt_minco_ros_test_'))
    os.environ['ROS_LOG_DIR'] = str(directory / 'ros_logs')
    scene = dict(bounds=[-3, 3, -3, 3], boxes=[], start=[-.2, 0], goal=[.2, 0])
    scene_path = directory / 'scene.json'
    scene_path.write_text(json.dumps(scene))
    children = []
    files = []
    try:
        def spawn(args, name):
            output = open(directory / name, 'w')
            files.append(output)
            process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            children.append(process)
        spawn(['roscore', '-p', str(port)], 'master.log')

        def master_ready():
            try:
                return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')[0] == 1
            except OSError:
                return False
        wait_for(master_ready)
        import numpy as np
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
        from quadrotor_msgs.msg import PositionCommand
        from std_msgs.msg import String
        from std_msgs.msg import Header
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        from std_srvs.srv import Trigger
        from visualization_msgs.msg import Marker
        rospy.init_node('rrt_adapter_test', anonymous=True, disable_signals=True)
        results, commands, paths, scenes, grids, voxels = [], [], [], [], [], []
        subscribers = [
            rospy.Subscriber('/UAV0/rrt/polynomial', String, lambda m: results.append(json.loads(m.data))),
            rospy.Subscriber('/UAV0/rrt/pos_cmd', PositionCommand, commands.append),
            rospy.Subscriber('/UAV0/rrt/path', RosPath, paths.append),
            rospy.Subscriber('/UAV0/rrt/scene', String, lambda m: scenes.append(json.loads(m.data))),
            rospy.Subscriber('/UAV0/rrt/grid_map', OccupancyGrid, grids.append),
            rospy.Subscriber('/UAV0/rrt/voxel_map', Marker, voxels.append)]
        odom_pub = rospy.Publisher('/UAV0/fast_lio/Odom_high_freq', Odometry, queue_size=1)
        goal_pub = rospy.Publisher('/UAV0/move_base_simple/goal' if args.cloud else '/UAV0/rrt/goal',
                                   PoseStamped, queue_size=1)
        map_odom_pub = rospy.Publisher('/UAV0/fast_lio/Odometry', Odometry, queue_size=1)
        cloud_pub = rospy.Publisher('/UAV0/fast_lio/cloud_registered', PointCloud2, queue_size=1)
        launch = (['real_cloud.launch', 'xmin:=-3', 'xmax:=3', 'ymin:=-3', 'ymax:=3',
                   'command_topic:=/UAV0/rrt/pos_cmd'] if args.cloud else
                  ['standalone.launch', 'scene_file:=' + str(scene_path)])
        spawn(['roslaunch', 'rrt_minco_adapter'] + launch, 'adapter.log')
        wait_for(lambda: goal_pub.get_num_connections() > 0 and odom_pub.get_num_connections() > 0)
        time.sleep(.3)
        goal = PoseStamped()
        goal.header.frame_id = 'UAV0/camera_init'
        goal.pose.position.x, goal.pose.position.z = .2, .6
        goal.pose.orientation.w = 1.
        goal.header.stamp = rospy.Time.now()
        goal_pub.publish(goal)
        wait_for(lambda: len(results) == 1)
        assert results[-1]['order'] == 0 and not commands
        height = .5 if args.cloud else .6
        if args.cloud:
            rospy.wait_for_service('/UAV0/rrt/freeze_map', timeout=10)
            freeze = rospy.ServiceProxy('/UAV0/rrt/freeze_map', Trigger)
            wait_for(lambda: cloud_pub.get_num_connections() > 0 and map_odom_pub.get_num_connections() > 0)
            map_odom = Odometry()
            map_odom.header.frame_id = 'UAV0/camera_init'
            map_odom.pose.pose.orientation.w = 1.
            cloud = point_cloud2.create_cloud_xyz32(
                Header(frame_id='UAV0/camera_init'),
                [(.05, 1.05, .35), (.15, 1.05, .65), (-4., 0., .5), (4., 0., .5)])
            for _ in range(12):
                map_odom.header.stamp = cloud.header.stamp = rospy.Time.now()
                map_odom_pub.publish(map_odom)
                cloud_pub.publish(cloud)
                time.sleep(.15)
            wait_for(lambda: grids and grids[-1].data.count(100) >= 2)
            assert freeze().success
            wait_for(lambda: scenes)
            assert scenes[-1]['z_min'] == .3 and scenes[-1]['z_max'] == .7
            assert scenes[-1]['boxes']
            grid = grids[-1]
            assert grid.header.frame_id == 'UAV0/camera_init'
            assert (grid.info.width, grid.info.height) == (60, 60)
            assert abs(grid.info.resolution - .1) < 1e-6
            assert (grid.info.origin.position.x, grid.info.origin.position.y) == (-3., -3.)
            wait_for(lambda: voxels and len(voxels[-1].points) >= 4)
            marker = voxels[-1]
            assert marker.header.frame_id == 'UAV0/camera_init'
            assert min(p.x for p in marker.points) < -3.
            assert max(p.x for p in marker.points) > 3.
            assert marker.type == Marker.CUBE_LIST
            assert marker.scale.x == marker.scale.y == marker.scale.z == .1
            assert len(marker.colors) == len(marker.points)
            # Frozen maps do not absorb later points on the route.
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(cloud.header, [(0., 0., .5)]))
            time.sleep(.1)
            assert freeze().success
            wait_for(lambda: len(scenes) == 2)
            assert scenes[0] == scenes[1]
            last_stamp = voxels[-1].header.stamp
            wait_for(lambda: voxels[-1].header.stamp > last_stamp)
            assert voxels[-1].points == marker.points
        odom = Odometry()
        odom.header.frame_id = 'UAV0/camera_init'
        odom.pose.pose.position.x, odom.pose.pose.position.z = -.2, height
        odom.pose.pose.orientation.w = 1.

        def publish_odom(_):
            odom.header.stamp = rospy.Time.now()
            odom_pub.publish(odom)
        timer = rospy.Timer(rospy.Duration(.02), publish_odom)
        time.sleep(.15)
        invalid = [('world', height, 0.), ('UAV0/camera_init', height, .3)]
        if not args.cloud:
            invalid.insert(1, ('UAV0/camera_init', .8, 0.))
        for frame, goal_height, speed in invalid:
            count = len(results)
            goal.header.frame_id, goal.pose.position.z = frame, goal_height
            odom.twist.twist.linear.x = speed
            time.sleep(.1)
            goal.header.stamp = rospy.Time.now()
            goal_pub.publish(goal)
            wait_for(lambda: len(results) > count)
            assert results[-1]['order'] == 0 and not commands
        odom.twist.twist.linear.x = 0.
        if args.cloud:
            goal.pose.position.z = 0.  # RViz XY clicks are lifted to the configured plane.
        time.sleep(.1)
        goal.header.stamp = rospy.Time.now()
        goal_pub.publish(goal)
        wait_for(lambda: any(r['order'] == 5 for r in results))
        result = next(r for r in results if r['order'] == 5)
        wait_for(lambda: len(commands) >= 5 and paths)
        durations = np.array(result['duration'])
        total = float(sum(durations))
        time.sleep(total + .2)
        assert paths[-1].header.frame_id == 'UAV0/camera_init'
        assert abs(paths[-1].poses[0].pose.position.x + .2) < 1e-6
        assert abs(paths[-1].poses[-1].pose.position.x - .2) < 1e-6
        start_ns = int(result['execution_start_ns'])
        for msg in commands:
            elapsed = (msg.header.stamp.to_nsec() - start_ns) * 1e-9
            t = max(0., min(elapsed, total))
            segment = min(int(np.searchsorted(np.cumsum(durations), t, side='right')), len(durations)-1)
            local_time = t - sum(durations[:segment])
            for axis in ['x', 'y', 'z']:
                coefficients = np.array(result['coef_' + axis]).reshape(-1, 6)[segment]
                for derivative, field in enumerate(['position', 'velocity', 'acceleration', 'jerk']):
                    expected = np.polyval(np.polyder(coefficients, derivative), local_time)
                    if elapsed >= total and derivative:
                        expected = 0.
                    assert abs(getattr(getattr(msg, field), axis) - expected) < 1e-6
            assert msg.trajectory_id == result['trajectory_id']
        assert abs(commands[-1].position.x - .2) < 1e-6
        assert abs(commands[-1].position.z - height) < 1e-6
        assert commands[-1].velocity.x == 0.
        publishers = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/test')[2][0]
        assert '/UAV0/planning/pos_cmd' not in dict(publishers)
        timer.shutdown()
        rospy.signal_shutdown('test complete')
        print('PASS: request rejection, planning, Path, polynomial/PositionCommand derivatives, endpoint hold, isolated topics')
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        for output in files:
            output.close()
        print('ROS test logs:', directory)


if __name__ == '__main__':
    main()
