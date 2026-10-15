#!/usr/bin/env python3
"""Exercise per-target swing gating and four output streams on localhost only."""
import math
import os
import signal
import socket
import subprocess
import tempfile
import time


def main():
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
            start(['roslaunch', 'mot_mapping', 'uav1_detection.launch',
                   'dynamic_mode:=swing-only', 'native_input_enabled:=true',
                   'shape_consistency_enabled:=true'], 'node.log')
            received = {k: {} for k in ('states', 'dynamic_cloud', 'dynamic_object_markers', 'dynamic_track_markers')}
            def collect(key, msg):
                stamp = msg.markers[0].header.stamp if key.endswith('markers') else msg.header.stamp
                received[key][stamp.to_nsec()] = msg
            subscriptions = [rospy.Subscriber('/UAV1/fapp/'+key, cls, lambda msg, k=key: collect(k, msg))
                             for key, cls in [('states', ObjectsStates), ('dynamic_cloud', PointCloud2),
                                              ('dynamic_object_markers', MarkerArray), ('dynamic_track_markers', MarkerArray)]]
            cloud = rospy.Publisher('/UAV1/fapp/cloud_registered_throttled', PointCloud2, queue_size=10)
            odom = rospy.Publisher('/UAV1/fast_lio/Odom_high_freq', Odometry, queue_size=10)
            time.sleep(3)
            deadline = time.monotonic()+10
            while (not cloud.get_num_connections() or not odom.get_num_connections()) and time.monotonic()<deadline:
                time.sleep(.1)
            assert cloud.get_num_connections() and odom.get_num_connections(), 'Missing input connections'
            def frame(x=None, second=None, bootstrap=False, rotated=False, seventh=False):
                header = Header(stamp=rospy.Time.now(), frame_id='UAV1/camera_init')
                pose = Odometry(header=header)
                pose.pose.pose.orientation.w = 1
                points = []
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
                rows = received['states'][key].states
                for name in ('dynamic_object_markers', 'dynamic_track_markers'):
                    assert sum(m.action == m.ADD for m in received[name][key].markers) == len(rows)
                assert bool(received['dynamic_cloud'][key].width) == bool(rows)
                assert all(abs(r.position.y) < .3 for r in rows), 'Linear second target leaked'
                time.sleep(.1)
                return len(rows)
            frame(2, bootstrap=True)
            for _ in range(5):
                assert frame(2) == 0
            # First outward leg alone must never activate avoidance.
            for i in range(12):
                assert frame(2+.06*i, 2+.06*i) == 0
            counts = [frame(2+.06*(11-i), 2+.06*(12+i)) for i in range(12)]
            assert any(counts), ('Reversal not confirmed', counts)
            for _ in range(16):
                last = frame(2)
            assert last == 0, 'Stopped target retained indefinitely'
            for _ in range(8):
                assert frame() == 0
            for i in range(10):
                assert frame(2+.06*i) == 0, 'Reacquisition inherited old reversal'
            # Six- and seven-point fragments changing shape must not accumulate
            # swing evidence, including before ordinary motion confirmation.
            for _ in range(8):
                assert frame() == 0
            for i, x in enumerate((2, 2.05, 2.10, 2.05, 2)):
                assert frame(x, rotated=i == 0) == 0, \
                    'Rejected early shape observation entered reversal history'
            for _ in range(8):
                assert frame() == 0
            for seventh in (False, True):
                for i in range(24):
                    x = 2+.06*(i if i < 12 else 23-i)
                    assert frame(x, rotated=bool(i % 2), seventh=seventh) == 0, \
                        'Inconsistent sparse shape confirmed as swing'
                for _ in range(8):
                    assert frame() == 0
            print('PASS per-target reversal gate, four outputs, stop, reacquisition and sparse shape rejection')
        finally:
            import shutil
            if os.path.exists(os.path.join(logs, 'node.log')):
                shutil.copyfile(os.path.join(logs, 'node.log'), '/tmp/swing_output_smoke_node.log')
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
