#!/usr/bin/env python3
"""Short synthetic communication check, never reads the replay labels."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np

ROOT = Path(__file__).resolve().parent
PORT = 15340
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:' + str(PORT), ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
                  ROS_HOME='/tmp/point_event_smoke', ROS_LOG_DIR='/tmp/point_event_smoke/log')
import rospy
from nav_msgs.msg import Odometry
from obj_state_msgs.msg import ObjectsStates
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray


def main():
    master = subprocess.Popen(['roscore', '-p', str(PORT)], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    received = {name: {} for name in ('states', 'dynamic_cloud', 'dynamic_object_markers', 'dynamic_track_markers')}
    try:
        for _ in range(80):
            if subprocess.run(['rosparam', 'list'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
                break
            time.sleep(.05)
        rospy.init_node('point_event_synthetic_smoke', anonymous=True)
        with (ROOT/'ros_smoke_node.log').open('w') as log:
            node = subprocess.Popen([str(ROOT/'event_node.py'), '_ros/lidar_topic:=/event_test/cloud', '_ros/odom_topic:=/event_test/odom',
                                     '_background_reference/capture_duration:=0', '_lidar/range_z:=2'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            subscriptions = []
            for name, kind in [('states', ObjectsStates), ('dynamic_cloud', PointCloud2), ('dynamic_object_markers', MarkerArray), ('dynamic_track_markers', MarkerArray)]:
                def callback(message, key=name):
                    stamp = message.markers[0].header.stamp if hasattr(message, 'markers') else message.header.stamp
                    received[key][stamp.to_nsec()] = message
                subscriptions.append(rospy.Subscriber('/UAV1/fapp/' + name, kind, callback))
            cloud_pub = rospy.Publisher('/event_test/cloud', PointCloud2, queue_size=10)
            odom_pub = rospy.Publisher('/event_test/odom', Odometry, queue_size=10)
            for _ in range(100):
                if cloud_pub.get_num_connections() and odom_pub.get_num_connections():
                    break
                assert node.poll() is None, 'Prototype exited'
                time.sleep(.05)
            time.sleep(.3)
            stamps = []
            base = rospy.Time.now().to_sec()
            for index, x in enumerate([2., 1., 1.1, 1.2, 1.3, 1.3, 1.3, 1.3]):
                stamp = rospy.Time.from_sec(base + index*.1)
                stamps.append(stamp.to_nsec())
                header = Header(stamp=stamp, frame_id='UAV1/camera_init')
                points = [[x, y, 0.] for y in [-.025, -.015, -.005, .005, .015, .025]]
                odom = Odometry(header=header)
                odom.pose.pose.orientation.w = 1.
                odom_pub.publish(odom)
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
                time.sleep(.1)
            time.sleep(.5)
            assert node.poll() is None, 'Prototype exited while processing'
            assert all(set(messages) == set(stamps) for messages in received.values()), 'Four output timestamp coverage differs'
            counts = [len(received['states'][stamp].states) for stamp in stamps]
            assert counts[:3] == [0, 0, 0] and counts[3:5] == [1, 1], counts
            assert counts[-1] == 0, counts
            for stamp in stamps:
                states = received['states'][stamp].states
                boxes = [m for m in received['dynamic_object_markers'][stamp].markers if m.action == m.ADD]
                arrows = [m for m in received['dynamic_track_markers'][stamp].markers if m.action == m.ADD]
                assert len(states) == len(boxes) == len(arrows)
                for state, box, arrow in zip(states, boxes, arrows):
                    p = np.array([[q.x, q.y, q.z] for q in box.points])
                    center = (p.min(axis=0) + p.max(axis=0))/2
                    np.testing.assert_allclose(center, [state.position.x, state.position.y, state.position.z], atol=1e-6)
                    np.testing.assert_allclose(center, [arrow.points[0].x, arrow.points[0].y, arrow.points[0].z], atol=1e-6)
            result = dict(pass_=True, scope='Eight synthetic frames; no recorded-data acceptance claimed', counts=counts, four_outputs_equal=True)
            (ROOT/'ros_smoke_result.json').write_text(json.dumps(result, indent=2))
            print(json.dumps(result))
    finally:
        for process in (node, master):
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)


if __name__ == '__main__':
    main()
