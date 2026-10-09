#!/usr/bin/env python3
"""Check configurable confirmation length with real 5/10 Hz publication."""
import argparse
import importlib.util
import math
import os
from pathlib import Path
import signal
import struct
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
    parser.add_argument('--background-reference', action='store_true')
    parser.add_argument('--reference-motion', action='store_true')
    parser.add_argument('--reference-strong-two-frame', action='store_true')
    parser.add_argument('--competition', action='store_true')
    parser.add_argument('--occlusion-startup', action='store_true')
    parser.add_argument('--time-evidence', action='store_true')
    parser.add_argument('--motion-window', type=float, default=.6)
    parser.add_argument('--reacquisition-gap', type=float)
    parser.add_argument('--voxel', type=float, default=.05)
    parser.add_argument('--density-split', action='store_true')
    parser.add_argument('--core-radius', type=float, default=.08)
    parser.add_argument('--core-fraction', type=float, default=.7)
    parser.add_argument('--global-assignment', action='store_true')
    parser.add_argument('--position-variance', type=float, default=.09)
    parser.add_argument('--translated-startup', action='store_true')
    parser.add_argument('--translated-fraction', type=float, default=.8)
    parser.add_argument('--reference-distance', type=float, default=.03)
    parser.add_argument('--reference-fraction', type=float, default=.5)
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
                '_background_reference/enabled:=' + str(args.background_reference).lower(),
                '_detection/reference_motion_enabled:=' + str(args.reference_motion).lower(),
                '_detection/reference_strong_two_frame_enabled:=' + str(args.reference_strong_two_frame).lower(),
                '_association/competition_enabled:=' + str(args.competition).lower(),
                '_detection/occlusion_startup_enabled:=' + str(args.occlusion_startup).lower(),
                '_detection/time_evidence_enabled:=' + str(args.time_evidence).lower(),
                '_detection/motion_window:=' + str(args.motion_window),
                '_detection/confirmed_reacquisition_enabled:=' + str(args.reacquisition_gap is not None).lower(),
                '_detection/reacquisition_max_gap:=' + str(args.reacquisition_gap or .6),
                '_clustering/voxel_size:=' + str(args.voxel),
                '_clustering/density_split_enabled:=' + str(args.density_split).lower(),
                '_clustering/core_radius:=' + str(args.core_radius),
                '_clustering/core_density_fraction:=' + str(args.core_fraction),
                '_association/global_assignment_enabled:=' + str(args.global_assignment).lower(),
                '_tracking/position_measurement_variance:=' + str(args.position_variance),
                '_detection/translated_startup_enabled:=' + str(args.translated_startup).lower(),
                '_detection/translated_startup_fraction:=' + str(args.translated_fraction),
                '_background_reference/distance:=' + str(args.reference_distance),
                '_background_reference/fraction:=' + str(args.reference_fraction),
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

        spec = importlib.util.spec_from_file_location('old_smoke',
                Path(__file__).with_name('perception_smoke.py'))
        smoke = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(smoke)
        if args.background_reference or args.reference_motion:
            for _ in range(math.ceil(5 * args.rate) + 1):
                publish(None, points=smoke.reference_backdrop())
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

        smoke.exercise_20cm_targets(publish, count)
        long_gap = max(.8, args.reacquisition_gap + .2 if args.reacquisition_gap is not None else .8)
        smoke.exercise_dense_startup(publish, count, args.frames,
                                     gap_frames=math.ceil(long_gap * args.rate))
        if args.competition and args.background_consistency:
            # Five fixed returns are visible in the raw history but do not
            # form a candidate. A sixth return later creates a closer static
            # competitor while the nine-point body continues moving.
            def competing_cap(center, y):
                return [(center + math.sqrt(.1 ** 2 - dy ** 2 - dz ** 2), y + dy, 1 + dz)
                        for dy in (-.06, 0, .06) for dz in (-.06, 0, .06)]

            fixed = competing_cap(2.15, -1.7)
            for _ in range(12):
                publish(None, points=backdrop)
            # Two earlier plane observations provide background evidence
            # without satisfying the detector's four-frame static cutoff.
            fixed_surface = [(2.22, -1.7 + y * .1, 1 + z * .1)
                             for y in range(-3, 4) for z in range(-3, 4)]
            for _ in range(2):
                publish(None, points=backdrop + fixed_surface)
            for _ in range(2):
                publish(None, points=backdrop + fixed[:5])
            for center in (1.8, 1.95, 2.1):
                stamp = publish(None, points=backdrop + fixed[:5] + competing_cap(center, -1.4))
            assert count(stamp) == 1, 'Competition regression did not establish a moving body'
            stamp = publish(None, points=backdrop + fixed[:6] + competing_cap(2.55, -1.4))
            assert count(stamp) >= 1, 'Static competitor erased the moving body'
            assert any(math.hypot(state.position.x - 2.62, state.position.y + 1.4) < .25
                       for state in received['states'][stamp.to_nsec()].states), \
                'Closer static fragment stole the moving body track'
        if args.local_planes:
            smoke.exercise_mixed_corner(publish, count)
        if args.background_reference or args.reference_motion:
            smoke.exercise_reference_targets(publish, count, args.reference_motion,
                                             args.background_reference, args.frames,
                                             args.reference_strong_two_frame,
                                             reacquisition_enabled=args.reacquisition_gap is not None,
                                             long_gap_frames=math.ceil(long_gap * args.rate),
                                             # Preserve the existing 0.6-second stop fixture
                                             # when comparing 5 Hz and 10 Hz inputs.
                                             stop_frames=math.ceil(.6 * args.rate))
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
        if args.time_evidence and args.rate == 10:
            # 0.16 m/s only travels 3.2 cm over three 10 Hz observations.
            # Confirmation must wait for measured displacement accumulated
            # inside the time window, rather than lowering the motion gate.
            fixed_speed = .16
            minimum_observations = max(args.frames, math.ceil(
                args.displacement * args.rate / fixed_speed - 1e-7) + 1)
            for sign in (1, -1):
                for _ in range(16):
                    publish(None, points=backdrop)
                slow_first = None
                for step in range(20):
                    stamp = publish(None, points=backdrop + sparse(
                        2 + sign * step * fixed_speed / args.rate, -.6))
                    number = count(stamp)
                    if step + 1 < minimum_observations:
                        assert number == 0, ('Slow target confirmed without measured displacement',
                                             sign, step + 1, minimum_observations)
                    if number and slow_first is None:
                        slow_first = step + 1
                    if slow_first is not None:
                        assert number == 1, ('0.16 m/s time-evidence target lost', sign, step)
                assert slow_first is not None and \
                    minimum_observations <= slow_first <= minimum_observations + 1, \
                    ('0.16 m/s confirmation outside measured time window', sign,
                     slow_first, minimum_observations)
                # Current input is required even for an already confirmed track.
                stamp = publish(None, points=backdrop)
                assert count(stamp) == 0, 'Time evidence emitted an unobserved slow target'
        if args.density_split:
            # A sampled physical 20 cm sphere is joined to a long rope under
            # the original DBSCAN radius. The box describes a true partition;
            # the cloud preserves the entire confirmed parent's real returns.
            def sphere_and_rope(center, rope_points=21):
                body = [(center + math.sqrt(.1 ** 2 - dy ** 2 - dz ** 2),
                         -.6 + dy, 1 + dz)
                        for dy in (-.06, -.03, 0, .03, .06)
                        for dz in (-.06, -.03, 0, .03, .06)]
                rope = [(center + .1, -.6, 1.10 + point * .04)
                        for point in range(rope_points)]
                return body + rope

            def voxel_centroids(points):
                def float32(value):
                    return struct.unpack('f', struct.pack('f', value))[0]

                inverse_leaf = float32(1 / float32(args.voxel))
                cells = {}
                for point in points:
                    observed = tuple(float32(value) for value in point)
                    key = tuple(math.floor(float32(value * inverse_leaf)) for value in observed)
                    cells.setdefault(key, []).append(observed)
                # PCL sums in float32; the small tolerance below covers that
                # summation rounding while checking actual voxel centroids.
                return [tuple(sum(point[axis] for point in cell) / len(cell) for axis in range(3))
                        for cell in cells.values()]

            def contains_point(points, expected):
                return any(sum((point[axis] - expected[axis]) ** 2 for axis in range(3)) < 1e-10
                           for point in points)

            for _ in range(16):
                publish(None, points=backdrop)
            split_first = None
            for step in range(10):
                center = 1.5 + step * .12
                target_points = sphere_and_rope(center)
                stamp = publish(None, points=backdrop + target_points)
                number = count(stamp)
                if number and split_first is None:
                    split_first = step + 1
                if step + 1 < args.frames:
                    assert number == 0, 'Partition bypassed real-observation confirmation'
                if step + 1 >= args.frames:
                    assert number == 1, ('20 cm sphere with rope was lost', step)
                    markers = [marker for marker in received['dynamic_object_markers'][
                        stamp.to_nsec()].markers if marker.action == marker.ADD]
                    assert len(markers) == 1 and markers[0].points, \
                        ('Sphere partition box missing', step)
                    minimum = [min(getattr(point, axis) for point in markers[0].points)
                               for axis in ('x', 'y', 'z')]
                    maximum = [max(getattr(point, axis) for point in markers[0].points)
                               for axis in ('x', 'y', 'z')]
                    assert max(high - low for low, high in zip(minimum, maximum)) < .35, \
                        ('Rope remains attached to published sphere', step)
                    assert math.hypot((minimum[0] + maximum[0]) / 2 - center - .08,
                                      (minimum[1] + maximum[1]) / 2 + .6) < .15, \
                        ('Partition moved sphere box away from measured body', step)
                    points = list(point_cloud2.read_points(received['dynamic_cloud'][
                        stamp.to_nsec()], field_names=('x', 'y', 'z'), skip_nans=True))
                    expected = voxel_centroids(target_points)
                    expected_body = voxel_centroids(target_points[:25])
                    assert len(points) == len(expected) and \
                        all(contains_point(expected, point) for point in points), \
                        ('Dynamic cloud differs from the current real voxel parent', step)
                    assert all(contains_point(points, point) for point in expected_body), \
                        ('Compact box discarded real sphere surface points', step)
                    assert any(point[2] > 1.5 for point in points), \
                        ('Dynamic cloud dropped the confirmed parent rope returns', step)
            assert split_first == args.frames, ('Sphere partition startup delayed', split_first)
            for _ in range(4):
                stamp = publish(None, points=backdrop)
                assert count(stamp) == 0, 'Partition emitted a target without a current candidate'
        if args.reacquisition_gap is not None:
            # The stationary fragment remains continuous while the confirmed
            # sphere is occluded. When only the sphere reappears, both previous
            # measured centers are inside the gate, but the sphere's is nearer.
            # Its published track must compete by distance rather than yielding
            # its actual observation history to the continuous wall fragment.
            from roscpp.srv import SetLoggerLevel
            logger_service = '/UAV1/fapp/mapping_node/set_logger_level'
            rospy.wait_for_service(logger_service, timeout=3)
            rospy.ServiceProxy(logger_service, SetLoggerLevel)('ros.mot_mapping', 'debug')

            def gap_cap(center):
                return [(center + math.sqrt(.1 ** 2 - dy ** 2 - dz ** 2), -2 + dy, 1 + dz)
                        for dy in (-.06, 0, .06) for dz in (-.06, 0, .06)]

            def diagnostic(prefix, stamp, x):
                for line in Path(log_path).read_text().splitlines():
                    if prefix + ' ' not in line:
                        continue
                    fields = line[line.index(prefix):].split()
                    if int(fields[1]) == stamp.to_nsec() and \
                            math.hypot(float(fields[2]) - x, float(fields[3]) + 2) < .10:
                        return fields
                raise AssertionError(('Missing track diagnostic', prefix, stamp.to_nsec(), x))

            for _ in range(math.ceil((args.reacquisition_gap + .4) * args.rate)):
                publish(None, points=backdrop)
            fixed_fragment = gap_cap(2.45)
            for _ in range(3):
                stamp = publish(None, points=backdrop + fixed_fragment)
                assert count(stamp) == 0, 'Unconfirmed fixed fragment was published'
            for step in range(max(5, args.frames + 2)):
                center = 1.2 + step * .15
                stamp = publish(None, points=backdrop + fixed_fragment + gap_cap(center))
                number = count(stamp)
                if step + 1 < args.frames:
                    assert number == 0, 'Short-gap fixture started without three real observations'
                else:
                    assert number == 1, ('Short-gap fixture did not establish sphere', step)
            previous = diagnostic('FAPP_TRACK', stamp, center + .08)
            assert int(previous[7]) >= args.frames and int(previous[12]) == 1, \
                ('Published sphere lacked actual observation history', previous)
            missing_frames = max(1, min(4, int(args.reacquisition_gap * args.rate) - 1))
            for _ in range(missing_frames):
                stamp = publish(None, points=backdrop + fixed_fragment)
                assert count(stamp) == 0, 'Occluded sphere was predicted into dynamic output'
            center += .15
            stamp = publish(None, points=backdrop + gap_cap(center))
            assert count(stamp) == 1, 'Continuous fragment stole the reappearing sphere'
            current = diagnostic('FAPP_TRACK', stamp, center + .08)
            reacquired = diagnostic('FAPP_REACQUIRE', stamp, center + .08)
            assert current[5] == previous[5] and reacquired[5] == previous[5], \
                ('Reappearance changed the published sphere track', previous, current, reacquired)
            assert int(reacquired[8]) == 1 and int(reacquired[9]) >= args.frames and \
                int(current[7]) >= args.frames and int(current[12]) == 1, \
                ('Reappearance reset measured history or skipped confirmation', reacquired, current)
            stamp = publish(None, points=backdrop)
            assert count(stamp) == 0, 'Reacquired track published without a current candidate'
            if args.shape_consistency:
                fragment_center = center + .08 + .15
                incompatible = [(fragment_center + dx, -2, 1 + dz)
                                for dx in (-.09, 0, .09) for dz in (-.09, 0, .09)]
                stamp = publish(None, points=backdrop + incompatible)
                assert count(stamp) == 0, 'Short-gap track accepted an incompatible planar fragment'
                shape = diagnostic('FAPP_SHAPE', stamp, fragment_center)
                rejection = diagnostic('FAPP_REJECT', stamp, fragment_center)
                assert int(shape[5]) >= 8 and int(shape[6]) >= 8 and \
                    float(shape[7]) < args.shape_fraction and int(shape[8]) == 0 and \
                    rejection[5] == 'incoherent_dense_shape', \
                    ('Short-gap shape rejection was not exercised', shape, rejection)
        assert node.poll() is None, 'Detector exited'
        extra_checks = '/0.16mps-time-window' if args.time_evidence and args.rate == 10 else ''
        if args.density_split:
            extra_checks += '/density-rope'
        if args.reacquisition_gap is not None:
            extra_checks += '/published-gap-wall-competition'
        print('PASS rate={} frames={} displacement={} first={} six-point/radial/flat/slow/wall/reversal/ego/output{}'.format(
            args.rate, args.frames, args.displacement, first, extra_checks), flush=True)
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
