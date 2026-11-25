#!/usr/bin/env python3
"""Save/freeze, stop mapper, reload on isolated master, and test coordinate bridge."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client
import sys


def wait_for(predicate, timeout=20):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.05)
    raise AssertionError('Timed out')


def main():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ['ROS_IP'] = '127.0.0.1'
    directory = Path(tempfile.mkdtemp(prefix='saved_map_test_'))
    os.environ['ROS_LOG_DIR'] = str(directory/'logs')
    processes, files = [], []
    try:
        def spawn(args, name):
            log = open(directory/name, 'w')
            files.append(log)
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
            return process
        spawn(['roscore', '-p', str(port)], 'master.log')
        def master_ready():
            try:
                return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')[0] == 1
            except OSError:
                return False
        wait_for(master_ready)
        import numpy as np
        import rospy
        from sensor_msgs import point_cloud2
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Header, String
        from std_srvs.srv import Trigger
        from mavros_msgs.msg import State
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
        from saved_map_math import rotation
        rospy.init_node('saved_map_test', disable_signals=True)
        path = directory/'site.json'
        mapper = spawn(['rosrun', 'rrt_minco_adapter', 'cloud_to_scene_node.py',
                        '__name:=test_mapper', '_bounds:=[-1,4,-1,4]', '_snapshot_input:=true',
                        '_start_clear_radius:=0.4', '_removed_voxels:=[[10,10,1]]',
                        '_filled_voxels:=[[80,80,4]]', '_save_path:='+str(path),
                        '~cloud:=/test/map'], 'mapper.log')
        cloud_pub = rospy.Publisher('/test/map', PointCloud2, queue_size=1, latch=True)
        rospy.wait_for_service('/test_mapper/save_map', timeout=20)
        save = rospy.ServiceProxy('/test_mapper/save_map', Trigger)
        assert not save().success
        rng = np.random.RandomState(8)
        points = rng.uniform([-2, -2, -.4], [4, 4, 1.5], (7000, 3))
        points[:2300, 0] = -1.5
        points[2300:4600, 1] = 3.5
        points = np.vstack([points, [[.025, .025, .125], [.075, .075, .175]]])
        wait_for(lambda: cloud_pub.get_num_connections() > 0)
        cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id='UAV0/camera_init'), points))
        time.sleep(1.)
        assert rospy.ServiceProxy('/test_mapper/freeze', Trigger)().success
        assert save().success
        assert not save().success  # Existing map cannot be silently overwritten.
        data = json.loads(path.read_text())
        assert data['scene']['operator_clear_region']['radius'] == .4
        from cloud_scene import CloudScene
        mask = CloudScene([-1, 4, -1, 4], removed_voxels=[[10, 10, 1]])
        assert len(data['localization_points']) == sum(not mask.is_removed(p) for p in points)
        assert not any(mask.is_removed(p) for p in data['localization_points'])
        assert not any(mask.is_removed(p) for p in data['voxels'])
        assert data['scene']['operator_removed_voxels'] == [[10, 10, 1]]
        assert data['scene']['inferred_wall_voxels'] == [[80, 80, 4]]
        assert any(abs(p[0]-7.05)<1e-6 and abs(p[1]-7.05)<1e-6 for p in data['voxels'])
        assert not any(abs(p[0]-7.05)<1e-6 and abs(p[1]-7.05)<1e-6 for p in data['localization_points'])
        # Frozen file must not change when new (empty) map arrives.
        cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id='UAV0/camera_init'), []))
        os.killpg(mapper.pid, signal.SIGINT)
        mapper.wait(timeout=10)
        spawn(['roslaunch', 'rrt_minco_adapter', 'saved_map_view.launch',
               'map_file:='+str(path), 'rviz:=false'], 'reload.log')
        rospy.wait_for_service('/UAV0/rrt/freeze_map', timeout=20)
        align = rospy.ServiceProxy('/UAV0/rrt/freeze_map', Trigger)
        assert not align().success  # no fresh disarmed state/scan
        scene, odoms, commands = [], [], []
        subs = [rospy.Subscriber('/UAV0/rrt/scene', String, scene.append),
                rospy.Subscriber('/UAV0/rrt/map_odom', Odometry, odoms.append),
                rospy.Subscriber('/UAV0/planning/pos_cmd', PositionCommand, commands.append)]
        state_pub = rospy.Publisher('/UAV0/mavros/state', State, queue_size=1, latch=True)
        scan_pub = rospy.Publisher('/UAV0/fast_lio/cloud_registered', PointCloud2, queue_size=1, latch=True)
        odom_pub = rospy.Publisher('/UAV0/fast_lio/Odom_high_freq', Odometry, queue_size=1, latch=True)
        cmd_pub = rospy.Publisher('/UAV0/rrt/map_pos_cmd', PositionCommand, queue_size=1, latch=True)
        wait_for(lambda: state_pub.get_num_connections() and scan_pub.get_num_connections() and cmd_pub.get_num_connections())
        yaw, shift = .1, np.array([.12, -.06, .03])
        source = (points-shift) @ rotation(yaw)
        state_pub.publish(State(armed=False))
        scan_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id='UAV0/camera_init'), source))
        time.sleep(.4)
        result = align()
        assert result.success, result.message
        assert not align().success  # no frame jumps while running
        wait_for(lambda: scene)
        assert json.loads(scene[-1].data)['frame_id'] == 'UAV0/saved_map'
        odom = Odometry()
        odom.header.frame_id = 'UAV0/camera_init'
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x = 1.
        odom_pub.publish(odom)
        wait_for(lambda: odoms)
        actual = odoms[-1]
        p = actual.pose.pose.position
        np.testing.assert_allclose([p.x, p.y, p.z], shift, atol=.01)
        np.testing.assert_allclose([actual.twist.twist.linear.x, actual.twist.twist.linear.y],
                                   [np.cos(yaw), np.sin(yaw)], atol=.01)
        cmd = PositionCommand()
        cmd.header.frame_id = 'UAV0/saved_map'
        cmd.position.x, cmd.position.y, cmd.position.z = 3., 3., .5
        cmd.velocity.x = cmd.acceleration.x = cmd.jerk.x = 1.
        cmd.yaw = .3
        cmd_pub.publish(cmd)
        wait_for(lambda: commands)
        output = commands[-1]
        p = output.position
        np.testing.assert_allclose([p.x, p.y, p.z], rotation(yaw).T @ (np.array([3., 3., .5])-shift), atol=.015)
        for v in (output.velocity, output.acceleration, output.jerk):
            np.testing.assert_allclose([v.x, v.y, v.z], rotation(yaw).T @ [1., 0., 0.], atol=.01)
        assert abs(output.yaw-.2) < .005
        assert output.header.frame_id == 'UAV0/camera_init'
        print('PASS: save, freeze, reload, startup alignment, odometry and inverse command bridge. Logs:', directory)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in reversed(processes):
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for stream in files:
            stream.close()


if __name__ == '__main__':
    main()
