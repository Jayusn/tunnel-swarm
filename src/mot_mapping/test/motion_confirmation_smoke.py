#!/usr/bin/env python3
"""Check configurable confirmation length with real 5/10 Hz publication."""
import argparse
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rate', type=int, choices=(5, 10), required=True)
    parser.add_argument('--frames', type=int, choices=(3, 4, 5), required=True)
    parser.add_argument('--displacement', type=float, default=.08)
    parser.add_argument('--min-speed', type=float, default=.15)
    parser.add_argument('--min-novel', type=float, default=.3)
    parser.add_argument('--tracked-novel', type=float)
    parser.add_argument('--min-cluster', type=int, default=6)
    parser.add_argument('--port', type=int, default=11620)
    parser.add_argument('--background-consistency', action='store_true')
    parser.add_argument('--local-planes', action='store_true')
    parser.add_argument('--local-before-association', action='store_true')
    parser.add_argument('--background-distance', type=float, default=.02)
    parser.add_argument('--background-fraction', type=float, default=.8)
    parser.add_argument('--background-before-association', action='store_true')
    parser.add_argument('--shape-consistency', action='store_true')
    parser.add_argument('--shape-distance', type=float, default=.05)
    parser.add_argument('--shape-fraction', type=float, default=.5)
    parser.add_argument('--planes', type=int, default=6)
    args = parser.parse_args()
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(args.port)
    os.environ['ROS_IP'] = '127.0.0.1'
    import rospy
    from nav_msgs.msg import Odometry
    from obj_state_msgs.msg import ObjectsStates
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header
    from visualization_msgs.msg import MarkerArray

    master = subprocess.Popen(['roscore', '-p', str(args.port)],
                              stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                              start_new_session=True)
    node = None
    try:
        time.sleep(2)
        rospy.init_node('motion_confirmation_smoke', anonymous=True)
        log_path = '/tmp/fapp_motion_smoke_{}_{}_{}_{}.log'.format(
            args.rate, args.frames, round(args.displacement * 100), args.port)
        with open(log_path, 'w') as log:
            node = subprocess.Popen([
                'rosrun', 'mot_mapping', 'mapping_node', '__name:=mapping_node',
                '_ros/lidar_topic:=/motion_test/cloud', '_ros/odom_topic:=/motion_test/odom',
                '_ros/frame_id:=UAV1/camera_init', '_dbscan/core_pts:=3',
                '_dbscan/tolerance:=0.02', '_dbscan/min_cluster:=' + str(args.min_cluster),
                '_dbscan/max_cluster:=800', '_detection/thresh_dist:=0.06',
                '_detection/thresh_var:=0.28',
                '_detection/motion_confirmation_frames:=' + str(args.frames),
                '_detection/motion_displacement:=' + str(args.displacement),
                '_detection/min_motion_speed:=' + str(args.min_speed),
                '_detection/min_motion_novel_fraction:=' + str(args.min_novel),
                '_detection/tracked_motion_novel_fraction:=' + str(
                    args.min_novel if args.tracked_novel is None else args.tracked_novel),
                '_background_consistency/enabled:=' + str(args.background_consistency).lower(),
                '_background_consistency/local_planes_enabled:=' + str(args.local_planes).lower(),
                '_background_consistency/local_post_association:=' + str(not args.local_before_association).lower(),
                '_background_consistency/distance:=' + str(args.background_distance),
                '_background_consistency/fraction:=' + str(args.background_fraction),
                '_background_consistency/post_association:=' + str(not args.background_before_association).lower(),
                '_detection/shape_consistency_enabled:=' + str(args.shape_consistency).lower(),
                '_detection/shape_consistency_distance:=' + str(args.shape_distance),
                '_detection/shape_consistency_fraction:=' + str(args.shape_fraction),
                '_background_plane/max_planes:=' + str(args.planes),
                '_association/geometry_enabled:=false', '_detection/depth_evidence_enabled:=false',
                '__ns:=/UAV1/fapp'], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
        received = {k: {} for k in ('states', 'dynamic_cloud', 'dynamic_object_markers',
                                   'dynamic_track_markers')}
        subscribers = []
        for key, msg_type in [('states', ObjectsStates), ('dynamic_cloud', PointCloud2),
                              ('dynamic_object_markers', MarkerArray),
                              ('dynamic_track_markers', MarkerArray)]:
            def callback(msg, key=key):
                stamp = msg.markers[0].header.stamp if key.endswith('markers') else msg.header.stamp
                received[key][stamp.to_nsec()] = msg
            subscribers.append(rospy.Subscriber('/UAV1/fapp/' + key, msg_type, callback))
        cloud_pub = rospy.Publisher('/motion_test/cloud', PointCloud2, queue_size=10)
        odom_pub = rospy.Publisher('/motion_test/odom', Odometry, queue_size=10)
        time.sleep(2)
        sequence = 0

        def publish(center, points=None, odom_x=0, odom_z=0, odom_yaw=0):
            nonlocal sequence
            # Exact observation intervals decouple motion math from host jitter.
            stamp = rospy.Time.from_sec(1000 + sequence / args.rate)
            sequence += 1
            header = Header(stamp=stamp, frame_id='UAV1/camera_init')
            odom = Odometry(header=header)
            odom.pose.pose.position.x = odom_x
            odom.pose.pose.position.z = odom_z
            odom.pose.pose.orientation.w = math.cos(odom_yaw / 2)
            odom.pose.pose.orientation.z = math.sin(odom_yaw / 2)
            odom_pub.publish(odom)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points or []))
            time.sleep(1 / args.rate)
            return stamp

        def count(stamp):
            s = stamp.to_nsec()
            deadline = time.monotonic() + 2
            while not all(s in received[k] for k in received) and time.monotonic() < deadline:
                time.sleep(.01)
            assert all(s in received[k] for k in received), ('missing output', s)
            number = len(received['states'][s].states)
            assert all(number == sum(m.action == m.ADD for m in received[k][s].markers)
                       for k in ('dynamic_object_markers', 'dynamic_track_markers'))
            assert bool(number) == bool(received['dynamic_cloud'][s].width)
            assert all(math.isfinite(v) for state in received['states'][s].states
                       for v in (state.velocity.x, state.velocity.y, state.velocity.z))
            return number

        def sparse(center, y=0):
            return [(center + x, y + dy, 1 + dz) for x, dy, dz in
                    [(.1, 0, 0), (.06, .08, 0), (.06, -.08, 0), (.06, 0, .08),
                     (.06, 0, -.08), (0, .0707106781, .0707106781)]]

        backdrop = [(4, y * .1, .4 + z * .1) for y in range(-12, 13) for z in range(15)]
        for _ in range(3):
            publish(None, points=backdrop)
        first = None
        for step in range(10):
            stamp = publish(None, points=backdrop + sparse(2 + step * .12, -.6))
            n = count(stamp)
            if n and first is None:
                first = step + 1
            if step + 1 < args.frames:
                assert n == 0, ('confirmed before required observations', args.frames, step + 1)
        assert first == args.frames, ('confirmation length not applied', args.frames, first)
        for _ in range(14):
            stamp = publish(None, points=backdrop + sparse(3.08, -.6))
        assert count(stamp) == 0, 'Stopped target retained dynamic confirmation'

        spec = importlib.util.spec_from_file_location('old_smoke',
                Path(__file__).with_name('perception_smoke.py'))
        smoke = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(smoke)
        smoke.exercise_20cm_targets(publish, count)
        smoke.exercise_dense_startup(publish, count, args.frames)
        if args.local_planes:
            smoke.exercise_mixed_corner(publish, count)
        # A flat 20 cm return must not become a background plane simply
        # because its own points are planar; six points cover the disk rim.
        for _ in range(12):
            publish(None, points=backdrop)
        for step in range(12):
            center = 1.5 + step * .08
            disk = [(center + .1 * math.cos(angle), -.6,
                     1 + .1 * math.sin(angle))
                    for angle in [i * math.pi / 3 for i in range(6)]]
            stamp = publish(None, points=backdrop + disk)
            if step >= 4:
                assert count(stamp) == 1, ('20 cm flat sparse target lost', step)
        # Satisfy both the displacement window and the two-scan 6 cm novelty
        # gate; adding confirmation frames does not extend novelty history.
        slow_speed = max(args.min_speed + .02,
                         args.rate * args.displacement / (args.frames - 1) * 1.1,
                         args.rate * .06 / 2 * 1.1)
        for sign in (1, -1):
            for _ in range(12):
                publish(None, points=backdrop)
            for step in range(18):
                stamp = publish(None, points=backdrop + sparse(
                    2 + sign * step * slow_speed / args.rate, -.6))
                if step >= 5:
                    assert count(stamp) == 1, ('20 cm slow target lost', sign, step)
        assert node.poll() is None, 'Detector exited'
        print('PASS rate={} frames={} displacement={} first={} six-point/radial/flat/slow/wall/reversal/ego/output'.format(
            args.rate, args.frames, args.displacement, first), flush=True)
    finally:
        for proc in (node, master):
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5)


if __name__ == '__main__':
    main()
