#!/usr/bin/env python3
"""Exercise the occlusion startup flag with real ROS clouds and odometry.

Every case first records the complete target path in the old map while all
returns are stationary. After three target-free callbacks, the target moves
over that path. Its old-map change test stays false, so only the new raw-ray
startup evidence can confirm it. Unknown rays and one supported history must
not start a track, including a zero novelty threshold.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


CASES = ('positive_enabled', 'positive_disabled', 'missing_history',
         'single_history', 'zero_threshold')


def run_case(args):
    enabled = args.case != 'positive_disabled'
    radial = args.case.startswith('positive_')
    novelty = 0 if args.case == 'zero_threshold' else .26
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:' + str(args.port),
                      ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
                      ROS_LOG_DIR=str(args.log_dir / args.case / 'roslogs'),
                      ROSCONSOLE_CONFIG_FILE=str(args.log_dir / 'rosconsole.conf'))
    with socket.socket() as guard:
        guard.bind(('127.0.0.1', args.port))
    import rospy
    from nav_msgs.msg import Odometry
    from obj_state_msgs.msg import ObjectsStates
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header
    from visualization_msgs.msg import MarkerArray

    case_dir = args.log_dir / args.case
    case_dir.mkdir(parents=True, exist_ok=True)
    master_log = (case_dir / 'master.log').open('w')
    node_log_path = case_dir / 'node.log'
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
        rospy.init_node('occlusion_startup_smoke', anonymous=True)
        # Registered points and the synthetic radar origin share the identity
        # pose; supply the origin explicitly rather than using UAV extrinsics.
        rospy.set_param('/UAV1/mapping/extrinsic_T', [0, 0, 0])
        command = [args.node, '__name:=mapping_node',
                   '_ros/lidar_topic:=/occlusion_test/cloud',
                   '_ros/odom_topic:=/occlusion_test/odom',
                   '_ros/frame_id:=UAV1/camera_init', '_dbscan/core_pts:=3',
                   '_dbscan/tolerance:=0.02', '_dbscan/min_cluster:=6',
                   '_dbscan/max_cluster:=800', '_detection/thresh_dist:=0.06',
                   '_detection/thresh_var:=0.28', '_detection/motion_confirmation_frames:=3',
                   '_detection/motion_displacement:=0.06', '_detection/min_motion_speed:=0.14',
                   '_detection/min_motion_novel_fraction:=' + str(novelty),
                   '_detection/occlusion_startup_enabled:=' + str(enabled).lower(),
                   '_detection/depth_evidence_enabled:=false',
                   '_background_reference/enabled:=false',
                   '_detection/reference_motion_enabled:=false',
                   '_detection/reference_strong_two_frame_enabled:=false',
                   '_background_consistency/enabled:=false',
                   '_background_consistency/local_planes_enabled:=false',
                   '_background_surface/enabled:=false', '_background_plane/max_planes:=0',
                   '_detection/shape_consistency_enabled:=false',
                   '_association/competition_enabled:=false', '_association/geometry_enabled:=false']
        with node_log_path.open('w') as node_log:
            node = subprocess.Popen(command, stdout=node_log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        received = {key: {} for key in ('states', 'dynamic_cloud',
                                       'dynamic_object_markers', 'dynamic_track_markers')}
        subscribers = []
        for key, msg_type in [('states', ObjectsStates), ('dynamic_cloud', PointCloud2),
                              ('dynamic_object_markers', MarkerArray),
                              ('dynamic_track_markers', MarkerArray)]:
            def callback(msg, key=key):
                stamp = msg.markers[0].header.stamp if key.endswith('markers') else msg.header.stamp
                received[key][stamp.to_nsec()] = msg
            subscribers.append(rospy.Subscriber('/UAV1/fapp/' + key, msg_type, callback))
        cloud_pub = rospy.Publisher('/occlusion_test/cloud', PointCloud2, queue_size=10)
        odom_pub = rospy.Publisher('/occlusion_test/odom', Odometry, queue_size=10)
        deadline = time.monotonic() + 5
        while (not cloud_pub.get_num_connections() or not odom_pub.get_num_connections()) and \
                time.monotonic() < deadline:
            time.sleep(.05)
        assert cloud_pub.get_num_connections() and odom_pub.get_num_connections(), 'Input subscription missing'
        sequence = 0

        def publish(points):
            nonlocal sequence
            # Exact 5 Hz observation stamps keep motion thresholds independent
            # of host scheduling while the real callbacks run at 5 Hz too.
            stamp = rospy.Time.from_sec(1000 + sequence * .2)
            sequence += 1
            header = Header(stamp=stamp, frame_id='UAV1/camera_init')
            odom = Odometry(header=header)
            odom.pose.pose.orientation.w = 1
            odom_pub.publish(odom)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
            time.sleep(.2)
            return stamp.to_nsec()

        def count(stamp):
            deadline = time.monotonic() + 2
            while not all(stamp in stream for stream in received.values()) and \
                    time.monotonic() < deadline:
                time.sleep(.01)
            assert all(stamp in stream for stream in received.values()), ('Missing output', stamp)
            state_msg = received['states'][stamp]
            number = len(state_msg.states)
            assert all(number == sum(marker.action == marker.ADD for marker in received[key][stamp].markers)
                       for key in ('dynamic_object_markers', 'dynamic_track_markers'))
            assert bool(number) == bool(received['dynamic_cloud'][stamp].width)
            assert all(math.isfinite(value) for state in state_msg.states
                       for value in (state.position.x, state.position.y, state.position.z,
                                     state.velocity.x, state.velocity.y, state.velocity.z))
            return number

        def cap(x, y):
            # Nine sparse visible-surface samples of a physical 20 cm sphere.
            return [(x + math.sqrt(.1 ** 2 - dy ** 2 - dz ** 2), y + dy, 1 + dz)
                    for dy in (-.06, 0, .06) for dz in (-.06, 0, .06)]

        # Dense historical returns at x=4 cover the current target rays even
        # when the sphere changes elevation angle during radial motion.
        wall = [(4, y * .03, z * .03) for y in range(-45, 46) for z in range(10, 110)]
        absent = [(-4, y * .1, z * .1) for y in range(-10, 11) for z in range(5, 15)]
        path = [(2, 0), (1.7, 0), (1.4, 0)] if radial else [(2, -.4), (2, 0), (2, .4)]
        stationary_path = [point for x, y in path for point in cap(x, y)]
        for _ in range(5):
            publish(wall + stationary_path)
        for _ in range(3):
            stamp = publish(wall if radial else absent)
            assert count(stamp) == 0, 'Target-free callback published a prediction'
        stamps, counts = [], []
        for index, (x, y) in enumerate(path):
            background = wall if radial or (args.case == 'single_history' and index == 1) else absent
            stamp = publish(background + cap(x, y))
            stamps.append(stamp)
            counts.append(count(stamp))
        assert counts == ([0, 0, 1] if args.case == 'positive_enabled' else [0, 0, 0]), \
            (args.case, 'Unexpected startup', counts)
        assert node.poll() is None, 'Detector exited'

        def diagnostic(prefix):
            for line in node_log_path.read_text().splitlines():
                if prefix + ' ' in line:
                    fields = line[line.index(prefix):].split()
                    if int(fields[1]) == stamps[-1]:
                        return fields
            raise AssertionError(('Missing diagnostic', prefix, stamps[-1], node_log_path))

        track = diagnostic('FAPP_TRACK')
        assert int(track[7]) == 3 and float(track[8]) >= .06 and float(track[9]) >= .14, \
            ('Actual motion observations missing', track)
        assert int(track[10]) == 0, ('Old map unexpectedly supplied startup', track)
        assert float(track[11]) >= .26, ('Raw-cloud novelty insufficient', track)
        if enabled:
            evidence = diagnostic('FAPP_EVIDENCE')
            startup = diagnostic('FAPP_OCCLUSION_STARTUP')
            dense = int(evidence[5])
            foreground = [int(evidence[10]), int(evidence[11])]
            assert dense >= 6 and int(startup[6]) == 1, ('Motion/points missing', evidence, startup)
            if args.case == 'positive_enabled':
                assert min(foreground) >= .26 * dense and int(startup[7]) == 1, \
                    ('Both histories failed to establish foreground', evidence, startup)
            else:
                assert float(startup[5]) == 0 and int(startup[7]) == 0, \
                    ('Unknown/partial history started motion', evidence, startup)
                if args.case == 'single_history':
                    assert foreground[0] == 0 and foreground[1] >= .26 * dense, \
                        ('Negative case did not isolate one positive history', evidence)
                else:
                    assert foreground == [0, 0], ('Missing-ray case acquired ray evidence', evidence)
        print('PASS case={} counts={} changed=0 real_observations=3 log={}'.format(
              args.case, counts, node_log_path), flush=True)
    finally:
        for proc in (node, master):
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5)
        master_log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=12690, help='First isolated master port.')
    parser.add_argument('--node', default=str(Path(__file__).resolve().parents[3] /
                                             'devel/lib/mot_mapping/mapping_node'))
    parser.add_argument('--case', choices=('all',) + CASES, default='all')
    parser.add_argument('--log-dir', type=Path)
    args = parser.parse_args()
    if args.log_dir is None:
        args.log_dir = Path(tempfile.mkdtemp(prefix='fapp_occlusion_startup_'))
    args.log_dir.mkdir(parents=True, exist_ok=True)
    (args.log_dir / 'rosconsole.conf').write_text('log4j.logger.ros=INFO\nlog4j.logger.ros.mot_mapping=DEBUG\n')
    assert Path(args.node).is_file(), args.node
    if args.case == 'all':
        for index, case in enumerate(CASES):
            subprocess.run([sys.executable, str(Path(__file__).resolve()), '--case', case,
                            '--port', str(args.port + index), '--node', args.node,
                            '--log-dir', str(args.log_dir)], check=True)
        print('PASS occlusion startup: flag/three observations/both histories/unknown/zero threshold', flush=True)
    else:
        run_case(args)


if __name__ == '__main__':
    main()
