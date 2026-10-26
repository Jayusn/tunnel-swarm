"""通过独立 ROS master 验证真实搜索管理器的换向和停滞行为。"""

import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import unittest

import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def stop_process(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


class LandingSweepProgressTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="landing_sweep_test_")
        cls.addClassCleanup(cls.directory.cleanup)
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            port = listener.getsockname()[1]
        os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
        os.environ["ROS_IP"] = "127.0.0.1"
        core_log = open(Path(cls.directory.name) / "roscore.log", "w")
        cls.addClassCleanup(core_log.close)
        core = subprocess.Popen(
            ["roscore", "-p", str(port)], stdout=core_log,
            stderr=subprocess.STDOUT, start_new_session=True,
        )
        cls.addClassCleanup(stop_process, core)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                rosgraph.Master("/sweep_test_startup").getPid()
                break
            except Exception:
                time.sleep(0.05)
        else:
            raise RuntimeError("测试 ROS master 启动失败")
        rospy.init_node("landing_sweep_progress_test", disable_signals=True)

    @classmethod
    def tearDownClass(cls):
        # nose 的旧版 unittest 适配器不会自动执行类级清理。
        cls.doClassCleanups()

    def setUp(self):
        self.name = "/" + self._testMethodName
        self.goals = []
        self.states = []
        self.verification_test = self._testMethodName.startswith("test_verification")
        self.altitude = 0.6 if self.verification_test else 1.7
        self.hint_publisher = rospy.Publisher(self.name + "/hint", PoseArray, queue_size=1)
        self.assigned_publisher = rospy.Publisher(self.name + "/assigned", PoseStamped, queue_size=1)
        self.addCleanup(self.hint_publisher.unregister)
        self.addCleanup(self.assigned_publisher.unregister)
        state_subscriber = rospy.Subscriber(self.name + "/state", String, lambda m: self.states.append(m.data))
        self.addCleanup(state_subscriber.unregister)
        self.goal_subscriber = rospy.Subscriber(
            self.name + "/goal", PoseStamped, self.goals.append, queue_size=100,
        )
        self.addCleanup(self.goal_subscriber.unregister)
        self.odom_publisher = rospy.Publisher(
            self.name + "/odom", Odometry, queue_size=1,
        )
        self.stage_publisher = rospy.Publisher(
            self.name + "/stage", String, queue_size=1, latch=True,
        )
        self.addCleanup(self.odom_publisher.unregister)
        self.addCleanup(self.stage_publisher.unregister)
        for key, value in {
            "odom_topic": self.name + "/odom",
            "stage_topic": self.name + "/stage",
            "subgoal_topic": self.name + "/goal",
            "verification_hint_topic": self.name + "/hint",
            "assigned_target_topic": self.name + "/assigned",
            "enable_single_front_hint_approach": False,
            "front_approach_distance": 0.0,
            "front_hint_initial_wait_sec": 1.0 if self.verification_test else 0.0,
            "front_yaw_scan_half_angle_rad": 0.0,
            "search_height": 1.7,
            "goal_reissue_sec": 0.1,
            "sweep_stall_timeout_sec": 4.0,
        }.items():
            rospy.set_param(self.name + "/" + key, value)
        command = [os.environ["LANDING_SWEEP_TEST_EXECUTABLE"]] if (
            "LANDING_SWEEP_TEST_EXECUTABLE" in os.environ
        ) else ["rosrun", "diff_planner", "landing_diff_search_manager"]
        node_log = open(Path(self.directory.name) / (self._testMethodName + ".log"), "w")
        self.addCleanup(node_log.close)
        process = subprocess.Popen(
            command + ["__name:=" + self.name.lstrip("/")],
            stdout=node_log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        self.addCleanup(stop_process, process)
        deadline = time.monotonic() + 10
        while not (self.odom_publisher.get_num_connections()
                   and self.stage_publisher.get_num_connections()):
            self.assertIsNone(process.poll(), "搜索管理器启动失败")
            self.assertLess(time.monotonic(), deadline, "搜索管理器连接超时")
            time.sleep(0.025)
        self.hold(0.0, 0.2)
        self.stage_publisher.publish(String(data="SEARCH_OUTSIDE_LANDING"))
        self.hold(0.0, 0.3)
        if self.verification_test:
            return
        self.assert_goal(0.0, -1.5)
        self.hold(-1.156, 0.15)
        self.assert_goal(0.0, 1.5)

    def hold(self, lateral, duration, forward=0.0):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x = forward
            odom.pose.pose.position.y = lateral
            odom.pose.pose.position.z = self.altitude
            odom.pose.pose.orientation.w = 1.0
            self.odom_publisher.publish(odom)
            time.sleep(0.025)

    def assert_goal(self, forward, lateral):
        self.assertTrue(self.goals, "未发布搜索航点")
        point = self.goals[-1].pose.position
        self.assertAlmostEqual(point.x, forward, places=4)
        self.assertAlmostEqual(point.y, lateral, places=4)

    def test_verification_keeps_height_and_yields_to_confirmed_assignment(self):
        hint = PoseArray()
        hint.poses = [Pose()]
        hint.header.frame_id = "world"
        hint.header.stamp = rospy.Time.now()
        hint.poses[0].position.x = 3.0
        hint.poses[0].position.z = 0.1
        hint.poses[0].orientation.w = 1.0
        self.hint_publisher.publish(hint)
        self.hold(0.0, 1.2)
        self.assert_goal(2.4, 0.0)
        self.assertAlmostEqual(self.goals[-1].pose.position.z, 0.6)
        self.assertIn("FRONT_ARUCO_VERIFY_APPROACH", self.states)
        self.assertNotIn("FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP", self.states)
        # 线索超过消息超时仍保留，不因为离开前视画面就抬升搜索。
        self.hold(0.0, 0.8, forward=1.0)
        self.assert_goal(2.4, 0.0)
        self.assertAlmostEqual(self.goals[-1].pose.position.z, 0.6)
        self.hold(0.0, 0.3, forward=2.4)
        goal_count = len(self.goals)
        self.hold(0.0, 0.3, forward=2.4)
        self.assertEqual(len(self.goals), goal_count, "close verification must not generate zero-length goals")
        # 两平台确认后的正式分配立即覆盖粗搜索点，恢复原有精降接近链路。
        target = PoseStamped()
        target.header.frame_id = "world"
        target.header.stamp = rospy.Time.now()
        target.pose.position.x = 4.0
        target.pose.position.z = 0.1
        target.pose.orientation.w = 1.0
        self.assigned_publisher.publish(target)
        self.hold(0.0, 0.3, forward=2.4)
        self.assert_goal(4.0, 0.0)
        self.assertIn("TWO_ARUCOS_ASSIGNED_APPROACH_FAR_PLATFORM", self.states)
        hint.header.stamp = rospy.Time.now()
        self.hint_publisher.publish(hint)
        self.hold(0.0, 0.2, forward=2.4)
        self.assert_goal(4.0, 0.0)

    def test_verification_clear_resumes_search_for_missing_second_platform(self):
        hint = PoseArray()
        hint.header.frame_id = "world"
        hint.header.stamp = rospy.Time.now()
        hint.poses = [Pose()]
        hint.poses[0].position.x = 3.0
        self.hint_publisher.publish(hint)
        self.hold(0.0, 1.2)
        self.assert_goal(2.4, 0.0)
        hint.poses = []
        hint.header.stamp = rospy.Time.now()
        self.hint_publisher.publish(hint)
        self.hold(0.0, 0.3)
        self.assertAlmostEqual(self.goals[-1].pose.position.z, 1.7)
        self.assertIn("FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP", self.states)

    def test_turnaround_retains_left_goal(self):
        # 2026-10-17 飞行换向段的实测距离，先远离目标再持续接近。
        distances = [
            2.656, 2.762, 2.823, 2.906, 2.953, 3.005, 3.043, 3.072,
            3.092, 3.097, 3.094, 3.070, 3.027, 2.967, 2.904, 2.836,
            2.713, 2.635, 2.538, 2.458,
        ]
        for distance in distances:
            self.hold(1.5 - distance, 0.25)
        self.assert_goal(0.0, 1.5)
        # 确认横扫能到达左端，下一行仍保留完整3米宽度。
        self.hold(1.5, 0.3)
        self.assert_goal(1.0, 1.5)
        self.hold(1.5, 0.3, forward=1.0)
        self.assert_goal(1.0, -1.5)

    def test_stationary_still_advances_after_timeout(self):
        self.hold(-1.156, 4.4)
        self.assert_goal(1.0, -1.156)

    def test_oscillation_does_not_keep_resetting_progress(self):
        self.hold(-0.9, 0.3)
        for _ in range(10):
            self.hold(-1.1, 0.25)
            self.hold(-0.9, 0.25)
        self.assertGreater(self.goals[-1].pose.position.x, 0.5)


if __name__ == "__main__":
    unittest.main()
