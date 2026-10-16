"""Decode actual MAVLink packets; no ROS node, FCU or motor output is started."""
import ast
import fcntl
import math
import os
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np
from mavros import mavlink as ros_mavlink
from mavros_msgs.msg import Mavlink
from nav_msgs.msg import Odometry
from pymavlink.dialects.v20 import common as mavlink2
import rospy
from tf.transformations import (quaternion_from_euler, quaternion_matrix,
                                quaternion_multiply)


source = Path(__file__).parents[1] / 'src' / 'laser_mid360.py'
tree = ast.parse(source.read_text())
sender_class = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                    and n.name == 'FastLioOdometrySender')
exec(compile(ast.Module(body=[sender_class], type_ignores=[]), str(source), 'exec'))


class OdometryPacketTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.reset_file = str(Path(self.directory.name) / 'counter')
        self.packets = []
        self.publisher = type('Publisher', (), {'publish': self.packets.append})()
        self.sender = FastLioOdometrySender(self.publisher, self.reset_file, 7, 191)

    def sample(self, seq=100, stamp=1234567890123456789, frame='UAV0/camera_init',
               angles=(0.0, 0.0, 0.0)):
        odom = Odometry()
        odom.header.seq = seq
        odom.header.stamp = rospy.Time(stamp // 10**9, stamp % 10**9)
        odom.header.frame_id = frame
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 2.0
        odom.pose.pose.position.z = 3.0
        q = quaternion_from_euler(*angles)
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        return odom

    def decode(self):
        return mavlink2.MAVLink(None).decode(
            bytearray(ros_mavlink.convert_to_bytes(self.packets[-1])))

    def test_frame_timestamp_fields_and_crc(self):
        sample = self.sample()
        self.assertTrue(self.sender.publish(sample))
        message = self.decode()
        self.assertEqual(message.get_type(), 'ODOMETRY')
        self.assertEqual(message.get_srcSystem(), 7)
        self.assertEqual(message.get_srcComponent(), 191)
        self.assertEqual(message.frame_id, mavlink2.MAV_FRAME_LOCAL_FRD)
        self.assertEqual(message.child_frame_id, mavlink2.MAV_FRAME_BODY_FRD)
        self.assertEqual(message.time_usec, sample.header.stamp.to_nsec() // 1000)
        self.assertEqual((message.x, message.y, message.z), (2.0, 1.0, -3.0))
        self.assertEqual(message.estimator_type, mavlink2.MAV_ESTIMATOR_TYPE_VISION)
        self.assertEqual(message.pose_covariance, [0.0] * 21)
        self.assertTrue(all(math.isnan(x) for x in [message.vx, message.vy, message.vz,
                            message.rollspeed, message.pitchspeed, message.yawspeed]))
        self.assertTrue(all(math.isnan(x) for x in message.velocity_covariance))

    def test_attitude_retains_existing_mavros_conversion(self):
        for angles in [(0, 0, 0), (0, 0, math.pi/2), (0.2, -0.3, -2.9)]:
            self.sender.publish(self.sample(seq=len(self.packets)+1, angles=angles))
            message = self.decode()
            q = message.q
            actual = quaternion_matrix([q[1], q[2], q[3], q[0]])
            # Existing ENU->NED / FLU->FRD conversion: yaw=pi/2-yaw,
            # roll is unchanged, pitch changes sign.
            expected = quaternion_matrix(quaternion_from_euler(
                angles[0], -angles[1], math.pi/2-angles[2]))
            np.testing.assert_allclose(actual, expected, atol=1e-6)
            self.assertAlmostEqual(sum(x*x for x in q), 1.0, places=6)

    def test_counter_persists_across_bridge_restart_and_wraps(self):
        self.sender.publish(self.sample())
        first = self.decode().reset_counter
        replacement = FastLioOdometrySender(self.publisher, self.reset_file)
        replacement.publish(self.sample())
        self.assertEqual(self.decode().reset_counter, (first+1) % 256)
        Path(self.reset_file).write_text('255')
        replacement = FastLioOdometrySender(self.publisher, self.reset_file)
        replacement.publish(self.sample())
        self.assertEqual(self.decode().reset_counter, 0)

    def test_duplicates_source_restart_and_source_frame_change(self):
        self.sender.publish(self.sample(seq=100, stamp=10**12))
        initial_counter = self.decode().reset_counter
        self.assertFalse(self.sender.publish(self.sample(seq=100, stamp=10**12)))
        self.assertEqual(len(self.packets), 1)
        self.sender.publish(self.sample(seq=101, stamp=10**12+10**7))
        self.assertEqual(self.decode().reset_counter, initial_counter)
        self.assertEqual(self.decode().get_seq(), 1)
        self.sender.publish(self.sample(seq=0, stamp=10**12+2*10**7))
        self.assertEqual(self.decode().reset_counter, initial_counter+1)
        self.sender.publish(self.sample(seq=1, stamp=10**12+3*10**7,
                                       frame='new/camera_init'))
        self.assertEqual(self.decode().reset_counter, initial_counter+2)
        self.sender.publish(self.sample(seq=2, stamp=10**12))
        self.assertEqual(self.decode().reset_counter, initial_counter+3)

    def test_sequence_wrap_is_not_a_source_restart(self):
        self.sender.publish(self.sample(seq=0xffffffff, stamp=10**12))
        counter = self.decode().reset_counter
        self.sender.publish(self.sample(seq=0, stamp=10**12+10**7))
        self.assertEqual(self.decode().reset_counter, counter)


if __name__ == '__main__':
    unittest.main()
