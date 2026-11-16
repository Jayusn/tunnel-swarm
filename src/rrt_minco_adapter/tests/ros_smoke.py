#!/usr/bin/env python3
"""Run with match_ws sourced; creates and tears down an isolated ROS master."""
import json
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
        from nav_msgs.msg import Odometry, Path as RosPath
        from quadrotor_msgs.msg import PositionCommand
        from std_msgs.msg import String
        rospy.init_node('rrt_adapter_test', anonymous=True, disable_signals=True)
        results, commands, paths = [], [], []
        subscribers = [
            rospy.Subscriber('/UAV0/rrt/polynomial', String, lambda m: results.append(json.loads(m.data))),
            rospy.Subscriber('/UAV0/rrt/pos_cmd', PositionCommand, commands.append),
            rospy.Subscriber('/UAV0/rrt/path', RosPath, paths.append)]
        odom_pub = rospy.Publisher('/UAV0/fast_lio/Odom_high_freq', Odometry, queue_size=1)
        goal_pub = rospy.Publisher('/UAV0/rrt/goal', PoseStamped, queue_size=1)
        spawn(['roslaunch', 'rrt_minco_adapter', 'standalone.launch',
               'scene_file:=' + str(scene_path)], 'adapter.log')
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
        odom = Odometry()
        odom.header.frame_id = 'UAV0/camera_init'
        odom.pose.pose.position.x, odom.pose.pose.position.z = -.2, .6
        odom.pose.pose.orientation.w = 1.

        def publish_odom(_):
            odom.header.stamp = rospy.Time.now()
            odom_pub.publish(odom)
        timer = rospy.Timer(rospy.Duration(.02), publish_odom)
        time.sleep(.15)
        for frame, height, speed in [('world', .6, 0.), ('UAV0/camera_init', .8, 0.),
                                     ('UAV0/camera_init', .6, .3)]:
            count = len(results)
            goal.header.frame_id, goal.pose.position.z = frame, height
            odom.twist.twist.linear.x = speed
            time.sleep(.1)
            goal.header.stamp = rospy.Time.now()
            goal_pub.publish(goal)
            wait_for(lambda: len(results) > count)
            assert results[-1]['order'] == 0 and not commands
        odom.twist.twist.linear.x = 0.
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
