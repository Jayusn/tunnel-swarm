#!/usr/bin/env python3
"""Run only synthetic trajectory bridges on rostest's isolated ROS master."""
import threading
import time
import unittest

import rospy
import rostest
from bspline.msg import Bspline
from geometry_msgs.msg import Point
from quadrotor_msgs.msg import PolynomialTraj, PolynomialTrajUav0
from std_msgs.msg import Int32
from traj_utils.msg import PolyTraj


class RoleBridgeTest(unittest.TestCase):
    def test_front_yaw_and_rear_wire_compatibility(self):
        front, rear = [], []
        front_event, rear_event = threading.Event(), threading.Event()

        def receive_front(message):
            front.append(message)
            front_event.set()

        def receive_rear(message):
            rear.append(message)
            rear_event.set()

        subscribers = [
            rospy.Subscriber('/role_test/front_output', PolynomialTrajUav0, receive_front),
            rospy.Subscriber('/role_test/rear_output', PolynomialTraj, receive_rear),
        ]
        front_pub = rospy.Publisher('/role_test/front_input', Bspline, queue_size=1)
        rear_pub = rospy.Publisher('/role_test/rear_input', PolyTraj, queue_size=1)
        brake_pub = rospy.Publisher('/role_test/brake', Int32, queue_size=1)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not all(
                p.get_num_connections() for p in (front_pub, rear_pub, brake_pub)):
            time.sleep(0.05)
        self.assertTrue(all(p.get_num_connections() for p in (front_pub, rear_pub, brake_pub)))

        spline = Bspline(order=3, traj_id=1, start_time=rospy.Time.now())
        spline.knots = [-3, -2, -1, 0, 1, 2, 3, 4]
        spline.pos_pts = [Point(i, 0, 1) for i in range(4)]
        spline.yaw_pts = [0.7] * 4
        spline.yaw_dt = 1.0
        front_pub.publish(spline)
        self.assertTrue(front_event.wait(5), 'No front trajectory received')
        output = front[-1]
        self.assertEqual(output.action, output.ACTION_ADD)
        self.assertTrue(output.has_yaw)
        self.assertEqual(output.header.frame_id, 'UAV0/camera_init')
        self.assertEqual(len(output.trajectory), len(output.yaw_trajectory))
        self.assertAlmostEqual(output.yaw_trajectory[0].data[-1], 0.7, places=6)

        # The rear producer and consumer still connect with the original wire type.
        message = PolyTraj(drone_id=1, traj_id=2, start_time=rospy.Time.now(), order=1)
        message.duration = [2]
        message.coef_x, message.coef_y, message.coef_z = [1, 0], [0, 0], [0, 1]
        rear_pub.publish(message)
        self.assertTrue(rear_event.wait(5), 'No rear trajectory received')
        self.assertEqual(rear[-1].trajectory_id, 2)
        self.assertEqual(rear[-1].header.frame_id, 'UAV1/camera_init')
        self.assertEqual(list(rear[-1].trajectory[0].data), [1, 0, 0, 0, 0, 1])
        self.assertNotEqual(PolynomialTraj._md5sum, PolynomialTrajUav0._md5sum)

        front_event.clear()
        brake_pub.publish(Int32(1))
        self.assertTrue(front_event.wait(5), 'No front brake notification received')
        self.assertEqual(front[-1].action, PolynomialTrajUav0.ACTION_ABORT)
        for subscriber in subscribers:
            subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('role_bridge_test')
    rostest.rosrun('autotrans_reference_bridge', 'role_bridge', RoleBridgeTest)
