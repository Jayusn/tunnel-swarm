#!/usr/bin/env python3
"""Exercise measured-motion confirmation against an observed background on localhost."""
import argparse
import math
import os
import signal
import socket
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--local-bidirectional', action='store_true')
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%s' % port, ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME', None)
    processes = []
    with tempfile.TemporaryDirectory(prefix='swing_smoke_') as logs:
        os.environ['ROS_LOG_DIR'] = logs
        def start(args, name):
            log = open(os.path.join(logs, name), 'w')
            proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            log.close()
            processes.append(proc)
        try:
            start(['roscore', '-p', str(port)], 'master.log')
            time.sleep(2)
            import rospy
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import PointCloud2
            from sensor_msgs import point_cloud2
            from std_msgs.msg import Header
            from visualization_msgs.msg import MarkerArray
            from obj_state_msgs.msg import ObjectsStates
            rospy.init_node('swing_output_smoke', anonymous=True)
            rospy.set_param('/UAV0/fapp/mapping_node/detection/object_motion_evidence_enabled', True)
            rospy.set_param('/UAV0/fapp/mapping_node/detection/object_motion_local_bidirectional_enabled', args.local_bidirectional)
            rospy.set_param('/UAV0/fapp/mapping_node/detection/motion_confirmation_duration', .5)
            start(['roslaunch', 'mot_mapping', 'uav0_detection.launch',
                   'dynamic_mode:=general', 'native_input_enabled:=true',
                   'shape_consistency_enabled:=true'], 'node.log')
            received = {k: {} for k in ('states', 'dynamic_cloud', 'dynamic_object_markers', 'dynamic_track_markers')}
            def collect(key, msg):
                stamp = msg.markers[0].header.stamp if key.endswith('markers') else msg.header.stamp
                received[key][stamp.to_nsec()] = msg
            subscriptions = [rospy.Subscriber('/UAV0/fapp/'+key, cls, lambda msg, k=key: collect(k, msg))
                             for key, cls in [('states', ObjectsStates), ('dynamic_cloud', PointCloud2),
                                              ('dynamic_object_markers', MarkerArray), ('dynamic_track_markers', MarkerArray)]]
            cloud = rospy.Publisher('/UAV0/fapp/cloud_registered_throttled', PointCloud2, queue_size=10)
            odom = rospy.Publisher('/UAV0/fast_lio/Odom_high_freq', Odometry, queue_size=10)
            time.sleep(3)
            deadline = time.monotonic()+10
            while (not cloud.get_num_connections() or not odom.get_num_connections()) and time.monotonic()<deadline:
                time.sleep(.1)
            assert cloud.get_num_connections() and odom.get_num_connections(), 'Missing input connections'
            frame_stamps = []
            def frame(x=None, second=None, bootstrap=False, rotated=False, seventh=False, spacing=.1):
                header = Header(stamp=rospy.Time.now(), frame_id='UAV0/camera_init')
                pose = Odometry(header=header)
                pose.pose.pose.orientation.w = 1
                points = [(4., y*.04, z*.04) for y in range(-25,101) for z in range(0,51)]
                for cx, cy in [(x, 0), (second, 1.5)]:
                    if cx is not None:
                        for dx, dy, dz in [(.07,0,0),(-.07,0,0),(0,.07,0),
                                           (0,-.07,0),(0,0,.07),(0,0,-.07)]:
                            if rotated:
                                dx, dy, dz = ((2*dx-dy+2*dz)/3,
                                              (2*dx+2*dy-dz)/3,
                                              (-dx+2*dy+2*dz)/3)
                            points.append((cx+dx, cy+dy, 1+dz))
                        if seventh:
                            points.append((cx, cy, 1))
                odom.publish(pose)
                cloud.publish(point_cloud2.create_cloud_xyz32(header, points))
                if bootstrap:
                    time.sleep(.15)
                    return 0
                key = header.stamp.to_nsec()
                # ApproximateTime may wait for the next odometry sample.
                time.sleep(.02)
                next_pose = Odometry()
                next_pose.header.stamp = header.stamp + rospy.Duration(.01)
                next_pose.header.frame_id = header.frame_id
                next_pose.pose.pose.orientation.w = 1
                odom.publish(next_pose)
                deadline = time.monotonic()+2
                while not all(key in values for values in received.values()) and time.monotonic()<deadline:
                    time.sleep(.005)
                assert all(key in values for values in received.values()), 'Missing output'
                frame_stamps.append(header.stamp.to_sec())
                rows = received['states'][key].states
                for name in ('dynamic_object_markers', 'dynamic_track_markers'):
                    assert sum(m.action == m.ADD for m in received[name][key].markers) == len(rows)
                assert bool(received['dynamic_cloud'][key].width) == bool(rows)
                assert all(r.position.x < 3.8 for r in rows), 'Stationary wall leaked'
                time.sleep(spacing)
                return len(rows)
            frame(2, bootstrap=True)
            for _ in range(5):
                assert frame(2) == 0
            counts = [frame(2+.06*i, 2+.06*i) for i in range(12)]
            assert any(counts[:3]), ('Linear confirmation later than third observation', counts)
            assert max(counts) == 2, ('Independent second target missing', counts)
            first = next(i for i,n in enumerate(counts) if n)
            latency = frame_stamps[-12+first] - frame_stamps[-12]
            assert latency <= .3, ('Dense linear confirmation latency', latency)
            print('DENSE_LINEAR_FIRST_CONFIRM_SECONDS', latency, flush=True)
            assert any(frame(2+.06*(11-i)) for i in range(12)), 'Reversal lost entirely'
            for _ in range(16):
                last = frame(2)
            assert last == 0, 'Stopped target retained indefinitely'
            for _ in range(8):
                assert frame() == 0
            assert frame(2) == 0, 'Reacquisition inherited old evidence'
            assert frame(2.06) == 0, 'Reacquisition needs two transitions'
            assert any(frame(2+.06*i) for i in range(2,7)), 'Reacquisition failed'
            for _ in range(8):
                assert frame() == 0
            for i in range(16):
                assert frame(2+.005*(i%2)) == 0, 'Static jitter confirmed'
            for period, samples in ((2., 40), (6., 70)):
                for _ in range(8):
                    frame()
                counts = [frame(2+.25*math.sin(2*math.pi*i*.1/period)) for i in range(samples)]
                print('SWING_PERIOD_SECONDS', period, 'HIT_FRAMES', sum(bool(n) for n in counts),
                      'TOTAL_FRAMES', samples, flush=True)
                if period == 2.:
                    assert any(counts), 'Fast swing missed entirely'
            for _ in range(8):
                frame()
            counts = [frame(2+.08*i, spacing=.3) for i in range(6)]
            first = next((i for i,n in enumerate(counts) if n), None)
            latency = None if first is None else frame_stamps[-6+first]-frame_stamps[-6]
            print('SPARSE_FIRST_CONFIRM_SECONDS', latency, 'COUNTS', counts, flush=True)
            print('PASS linear/multiple targets, reversal, stop, reacquisition, static jitter and four outputs')
        finally:
            import shutil
            if os.path.exists(os.path.join(logs, 'node.log')):
                shutil.copyfile(os.path.join(logs, 'node.log'), '/tmp/object_motion_output_smoke_node.log')
            for proc in reversed(processes):
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGTERM)
                        proc.wait(timeout=5)


if __name__ == '__main__':
    main()
