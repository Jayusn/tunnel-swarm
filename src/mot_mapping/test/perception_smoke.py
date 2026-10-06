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


def exercise_20cm_targets(publish, count):
    # Physical diameter 20 cm, with only six observed surface points.
    # Keep the earlier 14 cm six-point cases as a separate regression.
    def diameter20_points(center, y=0, z=1):
        # Samples from the visible cap of a sphere of radius 10 cm.
        # Its observed extent can be smaller than its physical diameter.
        return [(center + x, y + dy, z + dz) for x, dy, dz in
                [(.1, 0, 0), (.06, .08, 0), (.06, -.08, 0),
                 (.06, 0, .08), (.06, 0, -.08),
                 (0, .0707106781, .0707106781)]]

    backdrop = [(4, y * .1, .4 + z * .1)
                for y in range(-12, 13) for z in range(15)]
    for sign in [1, -1]:
        for _ in range(3):
            publish(None, points=backdrop)
        for step in range(10):
            center = 2 + sign * step * .12
            stamp = publish(None, points=backdrop + diameter20_points(center, -.6))
            if step >= 4:
                assert count(stamp) == 1, '20 cm radial moving target lost: ' + str((sign, step, count(stamp)))

    # Motion parallel to a nearby wall and repeated reversals near background.
    for _ in range(3):
        publish(None)
    nearby_wall = [(1 + x * .1, 2.24, .4 + z * .1)
                   for x in range(30) for z in range(15)]
    offsets = [0, .15, .30, .15]
    for step in range(16):
        stamp = publish(None, points=nearby_wall + diameter20_points(
            2 + offsets[step % 4], y=2.1), odom_x=step * .025, odom_yaw=step * .02)
        if step >= 4:
            assert count(stamp) == 1, '20 cm wall-adjacent reversal lost: ' + str((step, count(stamp)))


def exercise_dense_startup(publish, count, frames=3):
    # Nine samples on a physical 20 cm sphere exercise shape verification;
    # the separate six-point regressions deliberately bypass that check.
    wall = [(x * .1, .84, .4 + z * .1)
            for x in range(40) for z in range(15)]

    def cap(center):
        return [(center + math.sqrt(.1 ** 2 - dy ** 2 - dz ** 2), .7 + dy, 1 + dz)
                for dy in (-.06, 0, .06) for dz in (-.06, 0, .06)]

    for _ in range(12):
        publish(None, points=wall)
    first = None
    for step in range(10):
        stamp = publish(None, points=wall + cap(.5 + step * .12))
        if count(stamp) and first is None:
            first = step + 1
    assert first == frames, ('Dense target startup delayed', first, frames)
    for _ in range(4):
        stamp = publish(None, points=wall)
        assert count(stamp) == 0, 'Occlusion emitted a dense target prediction'
    first = None
    for step in range(8):
        stamp = publish(None, points=wall + cap(2.18 + step * .12))
        if count(stamp) and first is None:
            first = step + 1
        if step + 1 >= frames:
            assert count(stamp) == 1, ('Dense target reacquisition lost', step)
    assert first == frames, ('Dense target reacquisition delayed', first, frames)


def exercise_mixed_corner(publish, count):
    # Two fixed 40 cm faces: together they are not one plane. A previously
    # moving return must not lend its track to shifting samples of either face.
    corner = [(1.4 + x * .1, 0, .4 + z * .1)
              for x in range(5) for z in range(5)]
    corner += [(1.4, y * .1, .4 + z * .1)
               for y in range(5) for z in range(5)]
    for _ in range(6):
        publish(None, points=corner)
    for step in range(5):
        center_z = .17 + step * .12
        body = [(1.6 + math.sqrt(.1 ** 2 - dy ** 2 - dz ** 2), .24 + dy, center_z + dz)
                for dy in (-.06, 0, .06) for dz in (-.06, 0, .06)]
        stamp = publish(None, points=corner + body)
        if step >= 2:
            assert count(stamp) == 1, ('Moving body beside mixed corner was lost', step)
    for step in range(14):
        center = 1.55 + .1 * (step % 2)
        patch = [(center + dx, dy, .65 + dz)
                 for dx in (-.012, 0, .012) for dy in (-.012, .012)
                 for dz in (-.012, 0, .012)]
        stamp = publish(None, points=corner + patch)
        assert count(stamp) == 0, ('Mixed static corner inherited motion', step)


def main():
    port = os.environ.get('FAPP_SMOKE_PORT', '11431')
    master = subprocess.Popen(['roscore', '-p', port], stdout=subprocess.DEVNULL,
                              stderr=subprocess.STDOUT, start_new_session=True)
    launch = None
    rviz_processes = []
    try:
        time.sleep(2)
        rospy.init_node('fapp_smoke', anonymous=True)
        launch_log = open('/tmp/match_fapp_smoke_' + port + '.log', 'w')
        launch = subprocess.Popen(['roslaunch', 'mot_mapping', 'uav1_detection.launch',
                                   'input_rate:=20',
                                   'motion_displacement:=' + os.environ.get('FAPP_SMOKE_DISPLACEMENT', '0.08'),
                                   'min_motion_speed:=' + os.environ.get('FAPP_SMOKE_SPEED', '0.15'),
                                   'min_motion_novel_fraction:=' + os.environ.get('FAPP_SMOKE_NOVEL', '0.3'),
                                   'tracked_motion_novel_fraction:=' + os.environ.get('FAPP_SMOKE_TRACKED_NOVEL', os.environ.get('FAPP_SMOKE_NOVEL', '0.3')),
                                   'min_cluster:=' + os.environ.get('FAPP_SMOKE_MIN_CLUSTER', '6'),
                                   'geometry_enabled:=' + os.environ.get('FAPP_SMOKE_GEOMETRY', 'false'),
                                   'depth_evidence_enabled:=' + os.environ.get('FAPP_SMOKE_DEPTH', 'false'),
                                   'background_consistency_enabled:=' + os.environ.get('FAPP_SMOKE_BACKGROUND', 'false'),
                                   'background_local_planes_enabled:=' + os.environ.get('FAPP_SMOKE_LOCAL_PLANES', 'false'),
                                   'background_local_post_association:=' + os.environ.get('FAPP_SMOKE_LOCAL_POST', 'true'),
                                   'background_consistency_fraction:=' + os.environ.get('FAPP_SMOKE_BG_FRACTION', '0.8'),
                                   'background_consistency_post_association:=' + os.environ.get('FAPP_SMOKE_BACKGROUND_POST', 'true'),
                                   'shape_consistency_enabled:=' + os.environ.get('FAPP_SMOKE_SHAPE', 'false'),
                                   'shape_consistency_distance:=' + os.environ.get('FAPP_SMOKE_SHAPE_DISTANCE', '0.05'),
                                   'shape_consistency_fraction:=' + os.environ.get('FAPP_SMOKE_SHAPE_FRACTION', '0.5')], stdout=launch_log,
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

        def sparse_points(center, y=0):
            return [(center + x, y + dy, 1 + z) for x, dy, z in
                    [(.07, 0, 0), (-.07, 0, 0), (0, .07, 0),
                     (0, -.07, 0), (0, 0, .07), (0, 0, -.07)]]

        def publish(center, points=None, odom_x=0, odom_z=0, odom_yaw=0, interval=.25):
            stamp = rospy.Time.now()
            header = Header(stamp=stamp, frame_id='UAV1/camera_init')
            odom = Odometry(header=header)
            odom.pose.pose.orientation.w = math.cos(odom_yaw / 2)
            odom.pose.pose.orientation.z = math.sin(odom_yaw / 2)
            odom.pose.pose.position.x = odom_x
            odom.pose.pose.position.z = odom_z
            if points is None:
                points = [] if center is None else [
                    (center + x * .055, y * .055, 1 + z * .055)
                    for x in range(4) for y in range(4) for z in range(4)]
            odom_pub.publish(odom)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
            time.sleep(interval)
            return stamp

        def count(stamp):
            states = next(msg for msg in reversed(received['states'])
                          if msg.header.stamp == stamp)
            boxes = next(msg for msg in reversed(received['boxes'])
                         if msg.markers[0].header.stamp == stamp)
            tracks = next(msg for msg in reversed(received['tracks'])
                          if msg.markers[0].header.stamp == stamp)
            cloud = next(msg for msg in reversed(received['cloud'])
                         if msg.header.stamp == stamp)
            number = len(states.states)
            assert number == sum(m.action == m.ADD for m in boxes.markers)
            assert number == sum(m.action == m.ADD for m in tracks.markers)
            assert bool(number) == bool(cloud.width), 'Output sets disagree'
            assert all(math.isfinite(v) for marker in tracks.markers
                       for point in marker.points for v in (point.x, point.y, point.z)), \
                'Non-finite direction at a stop / turn'
            return number

        for center in [2, 2, 2, 2]:
            static_stamp = publish(center)
        assert count(static_stamp) == 0, 'Stationary object classified as moving'
        for center in [2.3, 2.45, 2.6, 2.75]:
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
        assert count(last_stamp) == 0, 'Occluded object emitted prediction'

        sparse_counts = []
        for center in [2.0, 2.12, 2.24, 2.36, 2.48]:
            stamp = publish(None, points=sparse_points(center))
            sparse_counts.append(count(stamp))
        assert any(sparse_counts[2:]), 'Six-point moving object lost'
        # A turn still has a current observation; disappearing never emits boxes.
        turn_stamp = publish(None, points=sparse_points(2.48))
        assert count(turn_stamp) == 1, 'Swing turn dropped immediately'
        for _ in range(10):
            stopped_stamp = publish(None, points=sparse_points(2.48))
            count(stopped_stamp)
        assert count(stopped_stamp) == 0, 'Stopped object remained dynamic'

        for shift in range(10):
            plane = [(1.5 + (x + shift) * .055, -.8 + y * .055, .5)
                     for x in range(24) for y in range(12)]
            stamp = publish(None, points=plane + sparse_points(2.48), odom_x=shift * .1)
            assert count(stamp) == 0, 'Thin static plane / moving UAV produced false boxes'

        for _ in range(3):
            publish(None)
        for step in range(7):
            points = sparse_points(1 + step * .15, .6) + sparse_points(1.4 + step * .15, .6)
            stamp = publish(None, points=points)
        assert count(stamp) == 2, 'Two detections updated a single track'

        # Registered static points stay fixed when the UAV translates/rotates.
        for _ in range(3):
            publish(None)
        for step in range(10):
            stamp = publish(None, points=sparse_points(2), odom_x=step * .1,
                            odom_yaw=step * .12)
            assert count(stamp) == 0, 'Ego motion was subtracted from registered points'
        for step in range(8):
            stamp = publish(None, points=sparse_points(2 + step * .12),
                            odom_x=step * .1, odom_yaw=step * .12)
            if step >= 4:
                assert count(stamp) == 1, 'Moving target lost during ego motion'

        # Similar three-frame displacement, different actual timestamp speed.
        for _ in range(3):
            publish(None)
        for step in range(8):
            stamp = publish(None, points=sparse_points(1.2 + step * .12, 1.5), interval=1)
            assert count(stamp) == 0, 'Motion below 0.15 m/s was confirmed'
        for _ in range(3):
            publish(None)
        for step in range(12):
            stamp = publish(None, points=sparse_points(2 + .15 * (step % 2), -1.5), interval=.5)
            if step >= 4:
                assert count(stamp) == 1, 'Three-frame reversal cancelled measured speed'

        # Uneven input timing must not erase the preceding fast segment.
        for _ in range(3):
            publish(None)
        publish(None, points=sparse_points(3, -2))
        publish(None, points=sparse_points(3.3, -2), interval=3)
        stamp = publish(None, points=sparse_points(3.45, -2))
        assert count(stamp) == 1, 'Input gap diluted an observed fast segment'

        # The ceiling and moving return are within the actual DBSCAN radius.
        # Together they exceed max_cluster=800; background removal must split
        # them before clustering without deleting the six object points.
        for _ in range(3):
            publish(None)
        ceiling = [(1 + x * .05, -.6 + y * .05, 1.16)
                   for x in range(42) for y in range(24)]
        mixed_counts = []
        for step in range(8):
            stamp = publish(None, points=ceiling + sparse_points(1.6 + step * .12))
            mixed_counts.append(count(stamp))
        assert all(n == 1 for n in mixed_counts[3:]), \
            'Moving six-point object connected to a large ceiling was lost'
        for _ in range(10):
            stamp = publish(None, points=ceiling + sparse_points(2.44))
        assert count(stamp) == 0, 'Object stopped beside a ceiling remained dynamic'

        for _ in range(3):
            publish(None)
        for step in range(8):
            center = 1.2 + step * .12
            rope = [(center, 0, 1.15 + n * .08) for n in range(12)]
            stamp = publish(None, points=sparse_points(center) + rope)
        assert count(stamp) == 1, 'Sparse attached rope made a moving object look planar'
        for step in range(12):
            # Vary the rope sampling while the body stays in the same position.
            rope = [(center, 0, 1.15 + n * .08) for n in range(4 + step % 8)]
            stamp = publish(None, points=sparse_points(center) + rope)
        assert count(stamp) == 0, 'Changing rope sampling made a stopped object move'

        # A compact static surface remains visible while its sampled dense
        # patch shifts. Center movement alone must not sustain a dynamic box.
        for _ in range(3):
            publish(None)
        for center in [2.7, 2.55, 2.4, 2.25]:
            stamp = publish(None, points=sparse_points(center))
        assert count(stamp) == 1, 'Moving object was not confirmed before stopping'
        anchors = [(2 + x * .1, y * .1, 1 + .08 * math.sin(x))
                   for x in range(4) for y in [-1, 0, 1]]
        for step in range(16):
            anchor = anchors[(step % 4) * 3 + 1]
            patch = [(anchor[0] + x, anchor[1] + y, anchor[2] + z)
                     for x in [-.03, 0, .03] for y in [-.03, 0, .03]
                     for z in [-.03, 0, .03]]
            stamp = publish(None, points=anchors + patch)
            if step >= 4:
                assert count(stamp) == 0, 'Static sampling changes sustained motion'

        # Repeat a swing through positions already seen by the background map.
        for offsets in [[0, .15, .30, .15], [0, .30]]:
            for _ in range(3):
                publish(None)
            for step in range(20):
                center = 2 + offsets[step % len(offsets)]
                stamp = publish(None, points=sparse_points(center))
                if step >= 4:
                    assert count(stamp) == 1, 'Repeated obvious swing lost after map update'

        # Sparse samples of one fixed surface must not inherit a moving track
        # when its dense patch alternates between different parts of the plane.
        for _ in range(3):
            publish(None)
        for center in [2.05, 2.2, 2.35, 2.5]:
            stamp = publish(None, points=[(x, y, z - .5)
                                         for x, y, z in sparse_points(center)])
        assert count(stamp) == 1, 'Motion setup for sampled surface was not detected'
        anchors = [(2.5 + x * .15, y * .15, .5)
                   for x in range(-2, 3) for y in range(-2, 3)]
        for step in range(16):
            center = 2.5 + [-.15, 0, .15, 0][step % 4]
            patch = [(center + x, y, .5)
                     for x in [-.06, 0, .06] for y in [-.06, 0, .06]]
            stamp = publish(None, points=anchors + patch)
            if step >= 3:
                assert count(stamp) == 0, 'Fixed surface fragments inherited motion'

        # A curved six-point body close to a wall still moves independently.
        for _ in range(3):
            publish(None)
        wall = [(1.1 + x * .1, .12, .65 + z * .1)
                for x in range(20) for z in range(8)]
        for step in range(10):
            stamp = publish(None, points=wall + sparse_points(1.4 + step * .12))
            if step >= 4:
                assert count(stamp) == 1, 'Moving body close to wall was rejected'

        exercise_20cm_targets(publish, count)
        exercise_dense_startup(publish, count)
        if os.environ.get('FAPP_SMOKE_LOCAL_PLANES', 'false') == 'true':
            exercise_mixed_corner(publish, count)

        def height_points(center, z, y):
            return [(center + x, y + dy, z + dz) for x, dy, dz in
                    [(.0625, 0, 0), (-.0625, 0, 0), (0, .0625, 0),
                     (0, -.0625, 0), (0, 0, .0625), (0, 0, -.0625)]]

        # Binary-exact coordinates make the boundary bottom exactly 1.5 m.
        for case, (height, odom_z, expected) in enumerate([
                (1.5625, 0, 1), (1.625, 0, 0), (1.53125, 0, 1),
                (1.625, .125, 1), (1.5, -.125, 0)]):
            for _ in range(3):
                publish(None, odom_z=odom_z)
            for step in range(8):
                # Separate locations keep earlier cases out of the static map.
                stamp = publish(None, points=height_points(2 + step * .125, height,
                                                         1 + case * .4),
                                odom_z=odom_z)
                if step >= 4:
                    assert count(stamp) == expected, \
                        'Relative whole-box height limit failed: ' + str((height, odom_z))
        if rviz_processes:
            _, _, graph = rospy.get_master().getSystemState()
            subscribers_by_topic = dict(graph[1])
            for topic in ['dynamic_cloud', 'dynamic_object_markers', 'dynamic_track_markers']:
                nodes = subscribers_by_topic.get('/UAV1/fapp/' + topic, [])
                assert all('/fapp_smoke_rviz_' + profile in nodes
                           for profile in ['uav1_lite', 'exp']), 'RViz display not subscribed: ' + topic
            print('PASS: both RViz profiles subscribe to FAPP clouds and markers')
        print('PASS: static, six-point motion, turn, stop, occlusion, thin planes, '
              'moving UAV, one-to-one tracks, ceiling-connected motion, attached rope, '
              'static sampling changes, repeated swing, sampled static surfaces, '
              'wall-adjacent motion, relative height boundary, world-frame speed, '
              'speed threshold, reversal and consistent stamped outputs')
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
