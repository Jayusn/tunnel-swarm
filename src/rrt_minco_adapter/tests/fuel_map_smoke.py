#!/usr/bin/env python3
"""Run with match_ws sourced; creates and tears down an isolated ROS master."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
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
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ['ROS_IP'] = '127.0.0.1'
    directory = Path(tempfile.mkdtemp(prefix='rrt_minco_ros_test_'))
    os.environ['ROS_LOG_DIR'] = str(directory / 'ros_logs')
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
        import rospy
        from nav_msgs.msg import Odometry, Path as RosPath
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Header, String
        from visualization_msgs.msg import Marker
        from std_srvs.srv import Trigger
        rospy.init_node('fuel_map_test', anonymous=True, disable_signals=True)
        clouds, markers, scenes, paths = [], [], [], []
        subscribers = [
            rospy.Subscriber('/UAV0/rrt/dry_run_path', RosPath, paths.append),
            rospy.Subscriber('/UAV0/rrt/fuel/occupancy_all', PointCloud2, clouds.append),
            rospy.Subscriber('/UAV0/rrt/voxel_map', Marker, markers.append),
            rospy.Subscriber('/UAV0/rrt/scene', String, lambda m: scenes.append(json.loads(m.data)))]
        cloud_pub = rospy.Publisher('/UAV0/fast_lio/cloud_registered', PointCloud2, queue_size=1)
        odom_pub = rospy.Publisher('/UAV0/fast_lio/Odometry', Odometry, queue_size=1)
        spawn(['roslaunch', 'rrt_minco_adapter', 'grid_map_view.launch', 'rviz:=false',
               'map_size_x:=12', 'map_size_y:=12'], 'mapping.log')
        wait_for(lambda: cloud_pub.get_num_connections() and odom_pub.get_num_connections(), 30)
        def observe(points, frames):
            for _ in range(frames):
                stamp = rospy.Time.now()
                odom = Odometry()
                odom.header = Header(stamp=stamp, frame_id='UAV0/camera_init')
                odom.pose.pose.orientation.w = 1.
                # Compensate configured LiDAR offset so ray origin is (.025,.025,.525).
                odom.pose.pose.position.x = .036
                odom.pose.pose.position.y = .04829
                odom.pose.pose.position.z = .48088
                odom_pub.publish(odom)
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(odom.header, points))
                time.sleep(.15)
        def occupied(x):
            if not clouds:
                return False
            return any(abs(p[0]-x) < .06 for p in point_cloud2.read_points(
                clouds[-1], field_names=('x', 'y', 'z'), skip_nans=True))
        near, far, outside = (1.025, .025, .525), (2.025, .025, .525), (-1.025, .025, .525)
        observe([near, outside], 1)
        time.sleep(.3)
        # FUEL initializes unknown cells at the occupancy threshold: the first hit
        # may occupy a cell; later free-space rays must be able to clear it.
        wait_for(lambda: occupied(near[0]))
        observe([near, outside], 14)
        wait_for(lambda: occupied(near[0]) and occupied(outside[0]))
        wait_for(lambda: markers and any(p.x < 0 for p in markers[-1].points))
        observe([far, outside], 20)
        wait_for(lambda: occupied(far[0]) and not occupied(near[0]))
        wait_for(lambda: markers and not any(abs(p.x-near[0]) < .08 for p in markers[-1].points))
        rospy.wait_for_service('/UAV0/rrt/freeze_map', 10)
        freeze = rospy.ServiceProxy('/UAV0/rrt/freeze_map', Trigger)
        assert freeze().success
        wait_for(lambda: scenes)
        frozen = scenes[-1]
        assert frozen['bounds'] == [0., 5., 0., 5.]
        assert frozen['boxes'] and all(box[0] >= 0 for box in frozen['boxes'])
        spawn(['rosrun', 'rrt_minco_adapter', 'ground_plan.py',
               '_output_dir:=' + str(directory / 'plan')], 'ground_plan.log')
        wait_for(lambda: paths, 30)
        assert paths[-1].header.frame_id == 'UAV0/camera_init'
        assert paths[-1].poses
        assert all(0 <= p.pose.position.x <= 5 and 0 <= p.pose.position.y <= 5
                   and abs(p.pose.position.z-.5) < 1e-9 for p in paths[-1].poses)
        saved_points = markers[-1].points
        observe([near, outside], 14)
        time.sleep(1.2)
        assert markers[-1].points == saved_points
        assert freeze().success
        wait_for(lambda: len(scenes) >= 2)
        assert scenes[-1] == frozen
        state = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/test')[2]
        assert not any('pos_cmd' in topic or 'position_cmd' in topic for topic, _ in state[0])
        assert not any('mavros' in service for service, _ in state[2])
        rospy.signal_shutdown('test complete')
        print('PASS: FUEL occupancy, ray clearing, outside-region display, snapshot replacement, freeze, RRT dry run, no flight outputs')
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
