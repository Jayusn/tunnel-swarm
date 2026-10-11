#!/usr/bin/env python3
"""Check real-message nearest-odom selection with a fixed ROS arrival fixture.

All HF messages arrive before the clouds; no interpolation, future-message
wait, bag replay, labels, or detector scoring is involved.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enabled', choices=('true', 'false'), required=True)
    parser.add_argument('--port', type=int, default=15560)
    parser.add_argument('--node', default=str(Path(__file__).resolve().parents[3] /
                                             'devel/lib/mot_mapping/mapping_node'))
    parser.add_argument('--log-dir', type=Path)
    args = parser.parse_args()
    enabled = args.enabled == 'true'
    log_dir = args.log_dir or Path(tempfile.mkdtemp(prefix='fapp_nearest_odom_'))
    log_dir.mkdir(parents=True, exist_ok=True)
    console = log_dir / 'rosconsole.conf'
    console.write_text('log4j.logger.ros=INFO\nlog4j.logger.ros.mot_mapping=DEBUG\n')
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:' + str(args.port),
                      ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
                      ROS_LOG_DIR=str(log_dir / 'roslogs'), ROSCONSOLE_CONFIG_FILE=str(console))
    with socket.socket() as guard:
        guard.bind(('127.0.0.1', args.port))
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header

    master_log = (log_dir / 'master.log').open('w')
    master = subprocess.Popen(['roscore', '-p', str(args.port)], stdout=master_log,
                              stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    try:
        for _ in range(100):
            if subprocess.run(['rosparam', 'list'], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL).returncode == 0:
                break
            time.sleep(.05)
        else:
            raise RuntimeError('Isolated ROS master failed')
        rospy.init_node('nearest_odom_smoke', anonymous=True)
        command = [args.node, '__name:=mapping_node', '_ros/lidar_topic:=/nearest_test/cloud',
                   '_ros/odom_topic:=/nearest_test/odom', '_background_plane/enabled:=false',
                   '_synchronization/nearest_available_odom_enabled:=' + args.enabled]
        node_log_path = log_dir / 'node.log'
        with node_log_path.open('w') as node_log:
            node = subprocess.Popen(command, stdout=node_log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        cloud_pub = rospy.Publisher('/nearest_test/cloud', PointCloud2, queue_size=10)
        odom_pub = rospy.Publisher('/nearest_test/odom', Odometry, queue_size=10)
        deadline = time.monotonic() + 5
        while not (cloud_pub.get_num_connections() and odom_pub.get_num_connections()):
            assert node.poll() is None and time.monotonic() < deadline, 'Input subscription missing'
            time.sleep(.05)
        assert rospy.get_param('/mapping_node/synchronization/nearest_available_odom_enabled') == enabled
        base = 1000_000_000_000
        cloud_offsets = (0, 80_000_000, 200_000_000, 300_000_000, 400_000_000)
        odom_offsets = (1_000_000, -1_000_000, 200_000_000, 300_000_000, 400_000_000)
        expected_offsets = (-1_000_000, 1_000_000, 200_000_000, 300_000_000, 400_000_000)
        real_positions = {}
        for index, offset in enumerate(odom_offsets, 1):
            stamp = base + offset
            message = Odometry(header=Header(stamp=rospy.Time(*divmod(stamp, 10**9))))
            position = (.01 * index, -.02 * index, .003 * index)
            message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = position
            message.pose.pose.orientation.w = 1
            real_positions[stamp] = position
            odom_pub.publish(message)
            time.sleep(.03)
        # Fixed messages are already available; this delay is fixture setup,
        # never a detector policy waiting for a future odometry message.
        time.sleep(.3)
        points = [(2 + x * .03, y * .03, .8 + z * .03)
                  for x in range(5) for y in range(5) for z in range(3)]
        for offset in cloud_offsets:
            header = Header(stamp=rospy.Time(*divmod(base + offset, 10**9)),
                            frame_id='UAV1/camera_init')
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
            time.sleep(.1)
        deadline = time.monotonic() + 3
        while node_log_path.read_text().count('FAPP_CALLBACK_TIME ') < len(cloud_offsets):
            assert node.poll() is None and time.monotonic() < deadline, 'Missing completed callback'
            time.sleep(.02)
        poses, selections = {}, {}
        for line in node_log_path.read_text().splitlines():
            match = re.search(r'FAPP_SYNC_POSE (\d+) (\d+) (.*)', line)
            if match:
                poses[int(match[1])] = (int(match[2]), list(map(float, match[3].split())))
            match = re.search(r'FAPP_SYNC_SELECTION (\d+) (\d+) (\d+) (\d+)', line)
            if match:
                selections[int(match[1])] = tuple(map(int, match.groups()[1:]))
        clouds = [base + offset for offset in cloud_offsets]
        assert set(poses) == set(clouds), ('Actual callback header coverage', poses)
        improved = 0
        for index, stamp in enumerate(clouds):
            header, pose = poses[stamp]
            assert header in real_positions, ('Selected message was never published', header)
            assert all(math.isclose(actual, expected, abs_tol=1e-8)
                       for actual, expected in zip(pose[:3], real_positions[header])), ('Pose synthesized', pose)
            assert pose[3:] == [0, 0, 0, 1], ('Orientation differs from real message', pose)
            if enabled:
                original, chosen, available = selections[stamp]
                assert original in real_positions and available == len(odom_offsets), selections
                assert header == chosen == base + expected_offsets[index], ('Nearest/tie selection', stamp, selections)
                improved += abs(chosen - stamp) < abs(original - stamp)
            else:
                assert header == base + odom_offsets[index], ('Original synchronizer pairing changed', stamp, header)
        assert set(selections) == (set(clouds) if enabled else set()), selections
        assert not enabled or improved >= 1, 'Fixture never exercised a closer cached message'
        result = dict(pass_=True, enabled=enabled, callback_headers=clouds, real_hf_positions=real_positions,
                      actual_poses=poses, selections=selections, strictly_improved_pairs=improved,
                      node_command=command, scope='Fixed synthetic ROS arrival fixture; no replay acceptance claim')
        (log_dir / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print('PASS nearest odom enabled={} callbacks={} improved={} log={}'.format(
              enabled, len(poses), improved, node_log_path), flush=True)
    finally:
        for process in (node, master):
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        master_log.close()


if __name__ == '__main__':
    main()
