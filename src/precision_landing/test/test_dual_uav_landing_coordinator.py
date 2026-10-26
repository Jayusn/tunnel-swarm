#!/usr/bin/env python3

import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Bool, Int32, String

from precision_landing.msg import LandingPlatform, LandingPlatformArray


class DualUavLandingCoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.ready = False
        self.uav0_id = -1
        self.uav1_id = -1
        self.uav0_target = None
        self.uav1_target = None
        self.released = False
        self.hint = None

        rospy.Subscriber("/test/assignments_ready", Bool, self._ready_callback)
        rospy.Subscriber("/test/uav0_assigned", Int32, self._uav0_id_callback)
        rospy.Subscriber("/test/uav1_assigned", Int32, self._uav1_id_callback)
        rospy.Subscriber("/test/uav0_target", PoseStamped, self._uav0_target_callback)
        rospy.Subscriber("/test/uav1_target", PoseStamped, self._uav1_target_callback)
        rospy.Subscriber("/test/release_uav1", Bool, self._release_callback)

        rospy.Subscriber("/test/verification_hint", PoseArray, self._hint_callback)
        self.search_candidates_pub = rospy.Publisher(
            "/test/search_candidates", LandingPlatformArray, queue_size=1)
        self.front_candidates_pub = rospy.Publisher(
            "/test/front_candidates", LandingPlatformArray, queue_size=1
        )
        self.exit_pub = rospy.Publisher(
            "/test/final_exit", PoseStamped, queue_size=1
        )
        self.state_pub = rospy.Publisher(
            "/test/search_state", String, queue_size=1
        )
        self.landing_request_pub = rospy.Publisher(
            "/test/uav0_landing_request", Bool, queue_size=1
        )

        self.assertTrue(
            self._wait_for(
                lambda: self.front_candidates_pub.get_num_connections() > 0
                and self.search_candidates_pub.get_num_connections() > 0
                and self.exit_pub.get_num_connections() > 0
                and self.state_pub.get_num_connections() > 0
                and self.landing_request_pub.get_num_connections() > 0
            ),
            "coordinator did not subscribe to test inputs",
        )

    def _hint_callback(self, message):
        self.hint = message if message.poses else None

    def _ready_callback(self, message):
        self.ready = message.data

    def _uav0_id_callback(self, message):
        self.uav0_id = message.data

    def _uav1_id_callback(self, message):
        self.uav1_id = message.data

    def _uav0_target_callback(self, message):
        self.uav0_target = message

    def _uav1_target_callback(self, message):
        self.uav1_target = message

    def _release_callback(self, message):
        self.released = message.data

    @staticmethod
    def _wait_for(predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return True
            rospy.sleep(0.02)
        return False

    @staticmethod
    def _candidate_array(stamp):
        message = LandingPlatformArray()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        for marker_id, x, y in ((31, 1.0, 2.0), (17, 3.0, 4.0)):
            platform = LandingPlatform()
            platform.header = message.header
            platform.id = marker_id
            platform.pose.header = message.header
            platform.pose.pose.position.x = x
            platform.pose.pose.position.y = y
            platform.pose.pose.orientation.w = 1.0
            platform.score = 1.0
            message.platforms.append(platform)
        return message

    def test_unconfirmed_search_then_immediate_nearest_pair_assignment(self):
        self.state_pub.publish(String(data="FRONT_ARUCO_FORWARD_APPROACH"))
        exit_pose = PoseStamped()
        exit_pose.header.stamp = rospy.Time.now()
        exit_pose.header.frame_id = "world"
        exit_pose.pose.orientation.w = 1.0
        self.exit_pub.publish(exit_pose)
        rospy.sleep(0.1)

        self.state_pub.publish(String(data="FRONT_ARUCO_INITIAL_WAIT"))
        self.front_candidates_pub.publish(
            self._candidate_array(rospy.Time.now() - rospy.Duration(2.0))
        )
        rospy.sleep(0.15)
        self.assertFalse(self.ready, "stale latched candidates must not assign")

        # 深度未确认的近点只能搜索，不能参与双机分配。
        uncertain = self._candidate_array(rospy.Time.now())
        uncertain.platforms = uncertain.platforms[1:]
        uncertain.platforms[0].id = 31
        uncertain.platforms[0].pose.pose.position.x = 1.0
        self.search_candidates_pub.publish(uncertain)
        self.assertTrue(self._wait_for(lambda: self.hint is not None))
        self.assertFalse(self.ready)
        self.assertAlmostEqual(self.hint.poses[0].position.x, 1.0)
        # 无新观测时保留线索；不是依靠图像每帧重发才能继续搜索。
        first_stamp = self.hint.header.stamp
        self.assertTrue(self._wait_for(lambda: self.hint.header.stamp > first_stamp))
        self.state_pub.publish(String(data="FRONT_ARUCO_VERIFY_APPROACH"))
        rospy.sleep(0.1)
        confirmed = self._candidate_array(rospy.Time.now())
        confirmed.platforms = confirmed.platforms[:1]
        self.front_candidates_pub.publish(confirmed)
        rospy.sleep(0.15)
        self.assertFalse(self.ready, "one confirmed plus one rough candidate must not assign")
        self.assertEqual(self.uav0_id, -1)
        self.assertEqual(self.uav1_id, -1)
        self.assertTrue(self._wait_for(lambda: self.hint is None), "confirmed-only hint must clear")
        uncertain.header.stamp = rospy.Time.now()
        uncertain.platforms[0].id = 99
        uncertain.platforms[0].pose.pose.position.x = 0.5
        self.search_candidates_pub.publish(uncertain)
        self.assertTrue(self._wait_for(lambda: self.hint is not None))
        # 一次确认多个：打乱到达顺序，较远的88不应占用名额。
        confirmed = self._candidate_array(rospy.Time.now())
        far = LandingPlatform()
        far.header = confirmed.header
        far.id = 88
        far.pose.header = confirmed.header
        far.pose.pose.position.x = 9.0
        far.pose.pose.orientation.w = 1.0
        confirmed.platforms = [far] + list(reversed(confirmed.platforms))
        self.front_candidates_pub.publish(confirmed)
        self.assertTrue(self._wait_for(lambda: self.ready), "assignment was not released")
        self.assertEqual(self.uav1_id, 31)
        self.assertEqual(self.uav0_id, 17)
        self.assertIsNotNone(self.uav1_target)
        self.assertIsNotNone(self.uav0_target)
        self.assertAlmostEqual(self.uav1_target.pose.position.x, 1.0)
        self.assertAlmostEqual(self.uav0_target.pose.position.x, 3.0)
        self.assertFalse(self.released, "assignment alone must not release UAV1")

        self.landing_request_pub.publish(Bool(data=True))
        self.assertTrue(
            self._wait_for(lambda: self.released),
            "UAV0 precision-landing request did not release UAV1",
        )


if __name__ == "__main__":
    rospy.init_node("dual_uav_landing_coordinator_test")
    rostest.rosrun(
        "precision_landing",
        "dual_uav_landing_coordinator_test",
        DualUavLandingCoordinatorTest,
    )
