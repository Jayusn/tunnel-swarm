#!/usr/bin/env python3
"""Exercise convergence switches using current ROS clouds on isolated masters.

The fixtures preserve registered world coordinates when the radar moves. Tests
inspect all four real outputs and diagnostic evidence, rather than reproducing
the detector's implementation in Python.
"""
import argparse
import importlib.util
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


FULL_CASES = ('six_radial_toward', 'six_radial_away', 'six_flat',
              'six_wall_reversal_ego', 'six_occlusion', 'new_corner')
BIDIRECTIONAL_CASES = ('bidir_preloaded_toward', 'bidir_preloaded_away',
                       'bidir_occluded', 'bidir_single_transition', 'bidir_gap')
CASES = {
    'default': ('sparse',) + FULL_CASES,
    'online': ('sparse',) + FULL_CASES,
    'depth': ('foreground', 'unknown', 'single_history', 'behind', 'ego_surface') + FULL_CASES,
    'competition': ('point_competition', 'no_competitor') + FULL_CASES,
    'combo': FULL_CASES,
    'strong2': ('two_foregrounds', 'previous_background', 'new_wall', 'new_wall_line',
                'planar_three_frame', 'line_three_frame'),
    'body': ('sparse',) + FULL_CASES,
    'bidir': ('sparse',) + FULL_CASES + BIDIRECTIONAL_CASES,
    'bidir_static': ('sparse',) + FULL_CASES + BIDIRECTIONAL_CASES,
    'body_bidir': ('sparse',) + FULL_CASES + BIDIRECTIONAL_CASES,
    'body_bidir_static': ('sparse',) + FULL_CASES + BIDIRECTIONAL_CASES,
    'partition': ('partition_children', 'partition_unknown',
                  'partition_unpublished_parent', 'sparse') + FULL_CASES,
}


def cap(x, y=0, z=1):
    return [(x + math.sqrt(.1 ** 2 - dy ** 2 - dz ** 2), y + dy, z + dz)
            for dy in (-.06, 0, .06) for dz in (-.06, 0, .06)]


def backdrop():
    return [(-4, y * .1, .4 + z * .1) for y in range(-12, 13) for z in range(15)]


def joined_bodies(x, include_child, z=1):
    parent = [(x + math.sqrt(.1 ** 2 - dy ** 2 - dz ** 2),
               -.6 + dy, z + dz)
              for dy in (-.08, -.04, 0, .04, .08)
              for dz in (-.04, 0, .04)]
    if include_child:
        parent += [(px, py + .5, pz) for px, py, pz in list(parent)]
        parent += [(x + .1, y, z) for y in (-.46, -.38, -.30, -.22)]
    return parent


def run_case(args):
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
    node_log_path = case_dir / 'node.log'
    master_log = (case_dir / 'master.log').open('w')
    master = subprocess.Popen(['roscore', '-p', str(args.port)], stdout=master_log,
                              stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if subprocess.run(['rosparam', 'list'], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL).returncode == 0:
                break
            time.sleep(.05)
        else:
            raise RuntimeError('Isolated ROS master did not start')
        rospy.init_node('convergence_smoke', anonymous=True)
        rospy.set_param('/UAV1/mapping/extrinsic_T', [0, 0, 0])
        complete_case = args.case in FULL_CASES
        reference = complete_case or args.method in ('online', 'strong2', 'combo', 'partition')
        consistency = complete_case or args.method in ('competition', 'combo')
        params = {
            'ros/lidar_topic': '/convergence_test/cloud',
            'ros/odom_topic': '/convergence_test/odom', 'ros/frame_id': 'UAV1/camera_init',
            'synchronization/nearest_available_odom_enabled': args.nearest_available_odom,
            'detection/dense_change_evidence_enabled': args.dense_change_evidence,
            'detection/published_startup_source_enabled': args.published_startup_source,
            'dbscan/core_pts': 3, 'dbscan/tolerance': .02,
            'dbscan/min_cluster': 6, 'dbscan/max_cluster': 800,
            'detection/thresh_dist': .06, 'detection/thresh_var': .28,
            'detection/motion_confirmation_frames': 3,
            'detection/motion_displacement': .06, 'detection/min_motion_speed': .14,
            'detection/min_motion_novel_fraction': .26,
            'detection/tracked_motion_novel_fraction': .20,
            'background_reference/enabled': reference,
            'background_reference/capture_duration': 5.0,
            'background_reference/distance': .04, 'background_reference/fraction': .5,
            'detection/reference_motion_enabled': args.method == 'strong2',
            'detection/reference_strong_two_frame_enabled': args.method == 'strong2',
            'background_consistency/enabled': consistency,
            'background_consistency/local_planes_enabled': consistency,
            'background_consistency/distance': .02,
            'background_consistency/fraction': .55,
            'background_consistency/post_association': True,
            'background_consistency/local_post_association': True,
            'background_surface/enabled': complete_case or args.method not in ('depth', 'competition'),
            'background_plane/max_planes': 6 if complete_case else
                (0 if args.method in ('depth', 'strong2') else 6),
            'detection/shape_consistency_enabled': False,
            'detection/depth_evidence_enabled': False,
            'detection/occlusion_startup_enabled': False,
            'detection/publish_measured_body_position_enabled': args.method in ('body', 'body_bidir', 'body_bidir_static'),
            'clustering/candidate_partition_enabled': args.candidate_partition or args.method == 'partition',
            'detection/bidirectional_ray_evidence_enabled': args.method in ('bidir', 'body_bidir', 'body_bidir_static'),
            'detection/bidirectional_static_suppression_enabled': args.method in ('bidir_static', 'body_bidir_static'),
            'association/competition_enabled': False,
            'association/geometry_enabled': False,
            'association/global_assignment_enabled': False,
            'background_online/enabled': args.method in ('online', 'combo'),
            'detection/strict_depth_evidence_enabled': args.method in ('depth', 'combo'),
            'association/point_source_competition_enabled': args.point_source_competition or
                args.method in ('competition', 'combo'),
        }
        command = [args.node, '__name:=mapping_node', '__ns:=/UAV1/fapp']
        if args.method == 'partition':
            params['detection/publish_measured_body_position_enabled'] = True
        command += ['_' + key + ':=' + (str(value).lower() if isinstance(value, bool) else str(value))
                    for key, value in params.items()]
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
        cloud_pub = rospy.Publisher('/convergence_test/cloud', PointCloud2, queue_size=10)
        odom_pub = rospy.Publisher('/convergence_test/odom', Odometry, queue_size=10)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and \
                (not cloud_pub.get_num_connections() or not odom_pub.get_num_connections()):
            time.sleep(.05)
        assert cloud_pub.get_num_connections() and odom_pub.get_num_connections(), 'Missing input subscribers'
        sequence = 0

        def publish(center=None, points=None, odom_x=0, odom_z=None, odom_yaw=0):
            nonlocal sequence
            stamp = rospy.Time.from_sec(1000 + sequence / 10)
            sequence += 1
            header = Header(stamp=stamp, frame_id='UAV1/camera_init')
            odom = Odometry(header=header)
            if odom_z is None:
                odom_z = 2 if args.case == 'partition_unknown' else \
                    (1 if args.case in ('six_radial_toward', 'six_radial_away') else 0)
            odom.pose.pose.position.x, odom.pose.pose.position.z = odom_x, odom_z
            odom.pose.pose.orientation.w = math.cos(odom_yaw / 2)
            odom.pose.pose.orientation.z = math.sin(odom_yaw / 2)
            odom_pub.publish(odom)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points or []))
            time.sleep(.1)
            return stamp

        def count(stamp):
            value = stamp.to_nsec()
            deadline = time.monotonic() + 2
            while not all(value in stream for stream in received.values()) and time.monotonic() < deadline:
                time.sleep(.01)
            assert all(value in stream for stream in received.values()), ('Missing output', value)
            states = received['states'][value].states
            number = len(states)
            assert all(number == sum(marker.action == marker.ADD for marker in received[key][value].markers)
                       for key in ('dynamic_object_markers', 'dynamic_track_markers'))
            assert bool(number) == bool(received['dynamic_cloud'][value].width)
            assert all(math.isfinite(component) for state in states
                       for component in (state.position.x, state.position.y, state.position.z,
                                         state.velocity.x, state.velocity.y, state.velocity.z))
            if params['detection/publish_measured_body_position_enabled']:
                boxes = [m for m in received['dynamic_object_markers'][value].markers if m.action == m.ADD]
                arrows = [m for m in received['dynamic_track_markers'][value].markers if m.action == m.ADD]
                for state, box, arrow in zip(states, boxes, arrows):
                    for axis in ('x', 'y', 'z'):
                        center = (min(getattr(p, axis) for p in box.points) +
                                  max(getattr(p, axis) for p in box.points)) / 2
                        assert abs(getattr(state.position, axis) - center) < 1e-6
                        assert abs(getattr(arrow.points[0], axis) - center) < 1e-6
            return number

        def diagnostic(prefix, stamp, center=None):
            lines = []
            for line in node_log_path.read_text().splitlines():
                if prefix + ' ' not in line:
                    continue
                fields = line[line.index(prefix):].split()
                if int(fields[1]) != stamp.to_nsec():
                    continue
                if center is None or math.hypot(float(fields[2]) - center[0],
                                                float(fields[3]) - center[1]) < .15:
                    lines.append(fields)
            assert lines, ('Missing diagnostic', prefix, stamp.to_nsec(), node_log_path)
            return lines[0]

        wall = backdrop()
        if (complete_case and args.case != 'new_corner') or \
                args.case in ('partition_children', 'partition_unpublished_parent'):
            # A real observed rear wall covers the six-point target's rays.
            # Keep this wall and every preceding target in raw history; in
            # particular radial-away targets may occlude their own old rays.
            wall = [(4, y * .03, z * .03)
                    for y in range(-85, 86) for z in range(10, 121)]
        if args.case == 'two_foregrounds':
            # A finite observed wall lies behind both physical targets. Its
            # dense samples cover the rays; novelty alone is not foreground.
            wall = [(-4, y * .03, z * .03)
                    for y in range(-25, 26) for z in range(10, 81)]
        if reference:
            reference_points = wall + (cap(1, -2.6) if args.case == 'previous_background' else [])
            fixed_parent = []
            if args.case == 'partition_unpublished_parent':
                fixed_parent = [(1.5 + math.sqrt(.15 ** 2 - dy ** 2 - dz ** 2),
                                 -.6 + dy, 1 + dz)
                                for dy in (-.12, -.08, -.04, 0, .04, .08, .12)
                                for dz in (-.12, -.08, -.04, 0, .04, .08, .12)
                                if dy ** 2 + dz ** 2 < .15 ** 2]
                reference_points += fixed_parent
            for _ in range(52):
                stamp = publish(points=reference_points)
                if sequence > 1:
                    assert count(stamp) == 0, 'Calibration published a dynamic candidate'
            assert 'FAPP_REFERENCE_READY ' in node_log_path.read_text(), 'Five-second reference not frozen'

        if args.case in ('partition_children', 'partition_unknown'):
            # The first body already has three actual observations. A second
            # body then enters the same DBSCAN parent through a sparse chain;
            # it must accumulate its own three measured component candidates.
            unknown = args.case == 'partition_unknown'
            for step in range(3):
                x, z = (1.5, 1 + step * .35) if unknown else (1.5 + step * .15, 1)
                stamp = publish(points=wall + joined_bodies(x, False, z))
                assert count(stamp) == (1 if step == 2 else 0), 'First body did not use three observations'
            parent_track = diagnostic('FAPP_TRACK', stamp, (x + .08, -.6))
            for step in range(4):
                # Without a rear return, vertical translation moves both
                # bodies outside their previous measured rays. The parent
                # still has ordinary three-frame motion; unknown rays must
                # not authorize the newly separated child.
                x, z = (1.5, 2.05 + step * .35) if unknown else (1.95 + step * .15, 1)
                stamp = publish(points=wall + joined_bodies(x, True, z))
                count(stamp)
                states = received['states'][stamp.to_nsec()].states
                parent_hit = any(abs(s.position.y + .6) < .15 and abs(s.position.x - x - .08) < .2
                                 and abs(s.position.z - z) < .15 for s in states)
                child_hit = any(abs(s.position.y + .1) < .15 and abs(s.position.x - x - .08) < .2
                                and abs(s.position.z - z) < .15 for s in states)
                assert parent_hit, ('Existing measured body was lost', step)
                assert child_hit == (step >= 2 and not unknown), \
                    ('Child lacks its own measured confirmation and ray evidence', args.case, step, child_hit)
                current_parent = diagnostic('FAPP_TRACK', stamp, (x + .08, -.6))
                child_track = diagnostic('FAPP_TRACK', stamp, (x + .08, -.1))
                assert current_parent[5] == parent_track[5] and child_track[5] != parent_track[5], \
                    ('Supplemental candidate changed parent track ownership', current_parent, child_track)
                assert int(child_track[7]) == min(step + 1, 3), \
                    ('Child borrowed prior parent observations', step, child_track)
                cloud = received['dynamic_cloud'][stamp.to_nsec()]
                xyz = list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True))
                assert len(xyz) == len(set(xyz)), 'Shared dynamic parent was appended repeatedly'
                assert any(abs(p[1] + .1) < .03 for p in xyz), 'Full actual parent points were dropped'
            if unknown:
                track = diagnostic('FAPP_TRACK', stamp, (x + .08, -.1))
                assert int(track[7]) >= 3, ('Unknown-ray child lacked real observations', track)
                assert float(track[8]) >= .06 and float(track[9]) >= .14 and track[10] == '1', \
                    ('Unknown-ray child lacked ordinary motion startup', track)
                transitions = [line[line.index('FAPP_BIDIRECTIONAL_TRANSITION'):].split()
                               for line in node_log_path.read_text().splitlines()
                               if 'FAPP_BIDIRECTIONAL_TRANSITION ' in line]
                child_transitions = [row for row in transitions if row[2] == track[5]]
                assert len(child_transitions) >= 2 and all(float(row[5]) == 0
                                                          for row in child_transitions[-2:]), \
                    ('Unknown-ray fixture contains actual foreground or vacated evidence', child_transitions)
                gates = [line[line.index('FAPP_SUPPLEMENTAL_EVIDENCE'):].split()
                         for line in node_log_path.read_text().splitlines()
                         if 'FAPP_SUPPLEMENTAL_EVIDENCE ' in line]
                gate = next(row for row in gates if int(row[1]) == stamp.to_nsec() and row[2] == track[5])
                assert gate[4] == '1' and gate[5] == '1' and float(gate[6]) >= .26 and gate[7] == '0', \
                    ('Unknown rays did not independently block a motion-confirmed child', gate)
            assert 'FAPP_CANDIDATE_PARTITION ' in node_log_path.read_text(), 'Candidate partition did not execute'
            stamp = publish(points=wall)
            assert count(stamp) == 0, 'Missing current components published a prediction'
        elif args.case == 'partition_unpublished_parent':
            # A real moving component has its own three observations and
            # repeatable foreground rays, but its densely sampled stationary
            # parent is explained by the frozen reference and is not published.
            for _ in range(3):
                stamp = publish(points=wall + fixed_parent)
                assert count(stamp) == 0, 'Frozen stationary parent was published'
            for step in range(4):
                x = 1.5 + step * .15
                child = cap(x, -.1)
                length = math.hypot(x - 1.5, .5)
                links = math.ceil(length / .08)
                bridge = [(1.6 + (x - 1.5) * index / links,
                           -.6 + .5 * index / links, 1)
                          for index in range(1, links)]
                stamp = publish(points=wall + fixed_parent + child + bridge)
                assert count(stamp) == 0, 'Supplemental child published without a current parent publication'
            track = diagnostic('FAPP_TRACK', stamp, (x + .08, -.1))
            assert int(track[7]) >= 3, ('Moving child lacked three actual observations', track)
            transitions = [line[line.index('FAPP_BIDIRECTIONAL_TRANSITION'):].split()
                           for line in node_log_path.read_text().splitlines()
                           if 'FAPP_BIDIRECTIONAL_TRANSITION ' in line]
            child_transitions = [row for row in transitions if row[2] == track[5]]
            assert len(child_transitions) >= 2 and all(float(row[5]) >= .26
                                                      for row in child_transitions[-2:]), \
                ('Unpublished-parent fixture lacks repeated real motion rays', child_transitions)
            gates = [line[line.index('FAPP_SUPPLEMENTAL_EVIDENCE'):].split()
                     for line in node_log_path.read_text().splitlines()
                     if 'FAPP_SUPPLEMENTAL_EVIDENCE ' in line]
            gate = next(row for row in gates if int(row[1]) == stamp.to_nsec() and row[2] == track[5])
            assert gate[4] == '0' and gate[5] == '1' and float(gate[6]) >= .26 and gate[7] == '1', \
                ('Unpublished parent did not independently block a motion-confirmed child', gate)
        elif args.case.startswith('six_'):
            def sphere(center_x, center_y=-.6, visible_side=1):
                # Six physical surface samples from a radius-10cm sphere;
                # the measured cap need not span its full physical diameter.
                return [(center_x + visible_side * x, center_y + y, 1 + z) for x, y, z in
                        [(.1, 0, 0), (.06, .08, 0), (.06, -.08, 0),
                         (.06, 0, .08), (.06, 0, -.08),
                         (0, .0707106781, .0707106781)]]

            def require_target(stamp, expected, step):
                number = count(stamp)
                states = received['states'][stamp.to_nsec()].states
                hit = any(math.sqrt((state.position.x - expected[0]) ** 2 +
                                    (state.position.y - expected[1]) ** 2 +
                                    (state.position.z - expected[2]) ** 2) < .25 and
                          max(state.size.x, state.size.y, state.size.z) < .65
                          for state in states)
                print('FULL_TARGET_AUDIT method={} case={} step={} count={} hit={} stamp={}'.format(
                    args.method, args.case, step, number, hit, stamp.to_nsec()), flush=True)
                if step < 2:
                    assert number == 0, ('Ordinary six-point target confirmed before three observations',
                                         args.method, args.case, step, number)
                else:
                    assert hit and number == 1, ('Six-point target not detected after three observations',
                                                args.method, args.case, step, number, hit)

            environment = wall
            if args.case == 'six_wall_reversal_ego':
                environment += [(1 + x * .1, 2.24, .4 + z * .1)
                                for x in range(30) for z in range(15)]
            for _ in range(8):
                stamp = publish(points=environment)
                assert count(stamp) == 0, 'Target-free background produced a dynamic box'
            if args.case in ('six_radial_toward', 'six_radial_away'):
                sign = -1 if args.case == 'six_radial_toward' else 1
                for step in range(12):
                    x = 2 + sign * step * .12
                    # The radar and sphere center share y=0/z=1. Positive-X
                    # targets expose their negative-X cap toward this radar;
                    # this is pure radial motion, not a changing view angle.
                    stamp = publish(points=environment + sphere(x, 0, visible_side=-1))
                    require_target(stamp, (x - .05, 0, 1), step)
            elif args.case == 'six_flat':
                for step in range(12):
                    x = 1.5 + step * .12
                    disk = [(x + .1 * math.cos(angle), -.6, 1 + .1 * math.sin(angle))
                            for angle in [index * math.pi / 3 for index in range(6)]]
                    stamp = publish(points=environment + disk)
                    require_target(stamp, (x, -.6, 1), step)
            elif args.case == 'six_wall_reversal_ego':
                offsets = (0, .15, .30, .15)
                for step in range(16):
                    x = 2 + offsets[step % 4]
                    stamp = publish(points=environment + sphere(x, 2.1),
                                    odom_x=step * .025, odom_yaw=step * .02)
                    require_target(stamp, (x + .05, 2.1, 1), step)
            elif args.case == 'six_occlusion':
                for step in range(8):
                    x = 2 - step * .12
                    stamp = publish(points=environment + sphere(x))
                    require_target(stamp, (x + .05, -.6, 1), step)
                for _ in range(9):
                    stamp = publish(points=environment)
                    assert count(stamp) == 0, 'Occlusion published a predicted six-point target'
                for step in range(8):
                    x = 2.5 - step * .12
                    stamp = publish(points=environment + sphere(x))
                    require_target(stamp, (x + .05, -.6, 1), step)
            stamp = publish(points=environment)
            assert count(stamp) == 0, 'Missing current six-point observation published a prediction'
        elif args.case == 'sparse':
            spec = importlib.util.spec_from_file_location('perception_cases',
                    Path(__file__).with_name('perception_smoke.py'))
            smoke = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(smoke)
            smoke.exercise_20cm_targets(publish, count)
            # The shared fixture defaults to four 5 Hz missing callbacks. At
            # 10 Hz extend the physical gap beyond the existing 0.6 s support.
            smoke.exercise_dense_startup(publish, count, frames=3, gap_frames=9)
            stamp = publish(points=[])
            assert count(stamp) == 0, 'Missing current observation published a prediction'
        elif args.case == 'new_corner':
            new_scene_stamp = rospy.Time.from_sec(1000 + sequence / 10).to_nsec()
            first_face = [(3.8, y * .1, .4 + z * .1)
                          for y in range(-10, 11) for z in range(15)]
            second_face = [(2.4 + x * .1, 1.0, .4 + z * .1)
                           for x in range(15) for z in range(15)]
            for points in (wall + first_face, wall + first_face + second_face):
                for _ in range(18):
                    stamp = publish(points=points)
                    assert count(stamp) == 0, 'New static corner produced a dynamic box'
            # Registered world surfaces stay fixed during radar translation and rotation.
            for step in range(8):
                stamp = publish(points=wall + first_face + second_face,
                                odom_x=step * .035, odom_yaw=step * .06)
                assert count(stamp) == 0, 'Radar motion moved a static world corner'
            if args.method in ('online', 'combo'):
                updates = [line[line.index('FAPP_ONLINE_UPDATE'):].split()
                           for line in node_log_path.read_text().splitlines()
                           if 'FAPP_ONLINE_UPDATE ' in line]
                assert any(int(update[1]) >= new_scene_stamp and int(update[2]) > 0
                           for update in updates), 'New visible static surfaces were never learned'
                assert any(int(update[6]) == 1 and float(update[7]) > 0
                           for update in updates), 'Rotation did not exercise paused background learning'
        elif args.case.startswith('bidir_'):
            # Preload all positions into the accumulated map, then remove the
            # object from raw history. The original startup latch must be false.
            far = [(4, y * .03, z * .03)
                   for y in range(-45, 46) for z in range(10, 110)]
            near = [(.8, y * .01, z * .01)
                    for y in range(-45, 46) for z in range(10, 160)]
            path = (1.4, 1.7, 2.0) if args.case == 'bidir_preloaded_away' else (2.0, 1.7, 1.4)
            preload = [p for x in path for p in cap(x, 0)]
            for _ in range(5):
                publish(points=far + preload)
            for _ in range(9):
                stamp = publish(points=far)
                assert count(stamp) == 0, 'Preloaded target published without current points'
            counts = []
            for step, x in enumerate(path):
                if args.case == 'bidir_gap' and step == 2:
                    stamp = publish(points=far)
                    assert count(stamp) == 0, 'Missing input candidate published a prediction'
                occluded = args.case == 'bidir_occluded' or \
                    (args.case == 'bidir_single_transition' and step < 2)
                stamp = publish(points=(near if occluded else far) + cap(x, 0))
                counts.append(count(stamp))
            positive = args.case in ('bidir_preloaded_toward', 'bidir_preloaded_away') and \
                params['detection/bidirectional_ray_evidence_enabled']
            assert counts == ([0, 0, 1] if positive else [0, 0, 0]), \
                ('Supplemental startup needs two consecutive valid transitions', args.case, counts)
            track = diagnostic('FAPP_TRACK', stamp, (path[-1] + .08, 0))
            decisions = [line[line.index('FAPP_BIDIRECTIONAL_DECISION'):].split()
                         for line in node_log_path.read_text().splitlines()
                         if 'FAPP_BIDIRECTIONAL_DECISION ' in line]
            decision = next(row for row in decisions
                            if int(row[1]) == stamp.to_nsec() and row[2] == track[5])
            assert decision[3] == '0', ('Original startup unintentionally covers fixture', decision)
            assert decision[4] == ('1' if positive else '0'), \
                ('Motion and static switches are not independent', decision)
        elif args.case in ('foreground', 'unknown', 'single_history', 'behind'):
            radial = args.case in ('foreground', 'behind')
            near_wall = [(4 if args.case != 'behind' else .8, y * .03, z * .03)
                         for y in range(-45, 46) for z in range(10, 110)]
            path = [(2, 0), (1.7, 0), (1.4, 0)] if radial else [(2, -.4), (2, 0), (2, .4)]
            stationary_path = [point for x, y in path for point in cap(x, y)]
            for _ in range(5):
                publish(points=near_wall + stationary_path)
            for _ in range(3):
                stamp = publish(points=near_wall if radial else wall)
                assert count(stamp) == 0, 'Target-free callback published a prediction'
            counts = []
            for index, (x, y) in enumerate(path):
                current_wall = near_wall if radial or \
                    (args.case == 'single_history' and index == 1) else wall
                stamp = publish(points=current_wall + cap(x, y))
                counts.append(count(stamp))
            assert counts == ([0, 0, 1] if args.case == 'foreground' else [0, 0, 0]), \
                (args.case, 'Strict depth startup', counts)
            track = diagnostic('FAPP_TRACK', stamp, (path[-1][0] + .08, path[-1][1]))
            evidence = diagnostic('FAPP_EVIDENCE', stamp, (path[-1][0] + .08, path[-1][1]))
            assert int(track[7]) == 3 and int(track[10]) == 0, ('Old map supplied unintended startup', track)
            if args.case == 'foreground':
                assert min(int(evidence[10]), int(evidence[11])) >= .26 * int(evidence[5]), evidence
            else:
                assert min(int(evidence[10]), int(evidence[11])) == 0, evidence
        elif args.case == 'ego_surface':
            surface = [(2, y * .06, .4 + z * .06) for y in range(-8, 9) for z in range(20)]
            for _ in range(5):
                publish(points=surface)
            for step in range(16):
                # Changing planar samples and actual radar pose cannot create foreground.
                points = [(2, y * .06 + (.02 if step % 2 else 0), .4 + z * .06)
                          for y in range(-8, 9) for z in range(20)]
                stamp = publish(points=points, odom_x=step * .025, odom_yaw=step * .04)
                assert count(stamp) == 0, 'Static sample changes with ego-motion started a track'
        elif args.case in ('point_competition', 'no_competitor'):
            fixed = cap(2.15, -1.7)
            fixed_surface = [(2.22, -1.7 + y * .1, 1 + z * .1)
                             for y in range(-3, 4) for z in range(-3, 4)]
            for _ in range(12):
                publish(points=wall)
            for _ in range(2):
                publish(points=wall + fixed_surface)
            for _ in range(2):
                publish(points=wall + fixed[:5])
            for center in (1.8, 1.95, 2.1):
                stamp = publish(points=wall + fixed[:5] + cap(center, -1.4))
            assert count(stamp) == 1, 'Competition fixture did not establish a moving body'
            previous = diagnostic('FAPP_TRACK', stamp, (2.18, -1.4))
            moving = cap(2.55, -1.4) if args.case == 'point_competition' else []
            stamp = publish(points=wall + fixed[:6] + moving)
            if moving:
                assert count(stamp) >= 1, 'Static fragment erased the moving body'
                current = diagnostic('FAPP_TRACK', stamp, (2.63, -1.4))
                assert current[5] == previous[5], ('Static fragment stole the published track', previous, current)
                assert 'FAPP_POINT_SOURCE_BLOCK ' in node_log_path.read_text(), \
                    'Point-source disambiguation did not execute'
            else:
                # With no independent competitor the original association remains available.
                current = diagnostic('FAPP_TRACK', stamp, (2.23, -1.7))
                assert current[5] == previous[5], ('No-competitor case changed original matching', previous, current)
        elif args.case in ('new_wall', 'new_wall_line', 'planar_three_frame', 'line_three_frame'):
            for _ in range(5):
                publish(points=wall)
            counts = []
            if args.case in ('new_wall_line', 'line_three_frame'):
                centers_y = (0.0, .35) if args.case == 'new_wall_line' else (0.0, .35, .70)
            else:
                centers_y = (0.0, .36) if args.case == 'new_wall' else (0.0, .36, .72)
            for center_y in centers_y:
                # The radar samples different 20 cm patches on the same fixed
                # world plane. Previously unseen patches can occlude the
                # farther wall, so bilateral rays alone are insufficient.
                if args.case in ('new_wall_line', 'line_three_frame'):
                    # Six actual points span 20 cm and remain in six distinct
                    # 5 cm voxels. A narrow static scan cannot define a plane.
                    patch = [(-2, center_y + .0125 + offset / math.sqrt(2),
                              .8375 + offset / math.sqrt(2))
                             for offset in (-.1, -.06, -.02, .02, .06, .1)]
                else:
                    patch = [(-2, center_y + .1 * math.cos(angle),
                              1 + .1 * math.sin(angle))
                             for angle in [index * math.pi / 3 for index in range(6)]]
                stamp = publish(points=wall + patch)
                counts.append(count(stamp))
            evidence = diagnostic('FAPP_EVIDENCE', stamp, (-2, centers_y[-1]))
            print('NEW_WALL_AUDIT counts={} evidence={}'.format(counts, evidence), flush=True)
            if args.case in ('new_wall', 'new_wall_line'):
                assert counts == [0, 0], ('New static wall bypassed ordinary three-frame confirmation', counts)
                exception = diagnostic('FAPP_TWO_FRAME_EVIDENCE', stamp, (-2, centers_y[-1]))
                if args.case == 'new_wall':
                    assert exception[8] == '1', ('New wall lacks historical fixed-plane support', exception)
                else:
                    assert int(evidence[5]) == 6 and int(evidence[10]) == 6 and int(evidence[11]) == 6, \
                        ('Line fixture lacks six measured bilateral foreground points', evidence)
                    assert exception[9] == '0', ('Degenerate geometry authorized a two-frame exception', exception)
            else:
                # Tangential motion of a sparse 20 cm planar target is also
                # ambiguous after two samples. Its ordinary confirmation
                # must remain available after the third real observation.
                assert counts == [0, 0, 1], ('Planar target lost ordinary three-frame motion', counts)
        elif args.case in ('two_foregrounds', 'previous_background'):
            for _ in range(5):
                publish(points=wall)
            centers = (-2.8, -2.44) if args.case == 'two_foregrounds' else (1.0, 1.36)
            y = 0 if args.case == 'two_foregrounds' else -2.6
            counts = []
            for center in centers:
                stamp = publish(points=wall + cap(center, y))
                counts.append(count(stamp))
                if args.case == 'two_foregrounds':
                    evidence = diagnostic('FAPP_EVIDENCE', stamp, (center + .08, y))
                    assert min(int(evidence[10]), int(evidence[11])) >= .5 * int(evidence[5]), \
                        ('Positive two-frame case lacks observed foreground', evidence)
            assert counts == ([0, 1] if args.case == 'two_foregrounds' else [0, 0]), \
                ('Strict two-frame needs foreground in both observations', args.case, counts)
            if args.case == 'previous_background':
                stamp = publish(points=wall + cap(1.72, y))
                assert count(stamp) == 1, 'Previous static sample prevented ordinary three-frame confirmation'
        else:
            raise AssertionError(args.case)
        assert node.poll() is None, 'Detector exited'
        print('PASS method={} case={} four_outputs=consistent current_observation=required log={}'.format(
              args.method, args.case, node_log_path), flush=True)
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
    parser.add_argument('--method', choices=tuple(CASES), required=True)
    parser.add_argument('--port', type=int, default=13100)
    parser.add_argument('--node', default=str(Path(__file__).resolve().parents[3] /
                                             'devel/lib/mot_mapping/mapping_node'))
    parser.add_argument('--case', default='all')
    parser.add_argument('--point-source-competition', action='store_true')
    parser.add_argument('--candidate-partition', action='store_true')
    parser.add_argument('--nearest-available-odom', action='store_true')
    parser.add_argument('--dense-change-evidence', action='store_true')
    parser.add_argument('--published-startup-source', action='store_true')
    parser.add_argument('--log-dir', type=Path)
    args = parser.parse_args()
    assert args.case == 'all' or args.case in CASES[args.method], (args.method, args.case)
    assert Path(args.node).is_file(), args.node
    if args.log_dir is None:
        args.log_dir = Path(tempfile.mkdtemp(prefix='fapp_convergence_'))
    args.log_dir.mkdir(parents=True, exist_ok=True)
    (args.log_dir / 'rosconsole.conf').write_text('log4j.logger.ros=INFO\nlog4j.logger.ros.mot_mapping=DEBUG\n')
    if args.case == 'all':
        failures = []
        for index, case in enumerate(CASES[args.method]):
            command = [sys.executable, str(Path(__file__).resolve()), '--method', args.method,
                                     '--case', case, '--port', str(args.port + index), '--node', args.node,
                                     '--log-dir', str(args.log_dir)]
            if args.point_source_competition:
                command.append('--point-source-competition')
            if args.candidate_partition:
                command.append('--candidate-partition')
            if args.nearest_available_odom:
                command.append('--nearest-available-odom')
            if args.dense_change_evidence:
                command.append('--dense-change-evidence')
            if args.published_startup_source:
                command.append('--published-startup-source')
            result = subprocess.run(command)
            if result.returncode:
                failures.append((case, result.returncode))
        assert not failures, ('Convergence regression failures', args.method, failures)
        print('PASS convergence method=' + args.method, flush=True)
    else:
        run_case(args)


if __name__ == '__main__':
    main()
