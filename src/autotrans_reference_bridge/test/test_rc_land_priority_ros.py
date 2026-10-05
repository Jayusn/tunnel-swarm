#!/usr/bin/env python3
"""隔离 ROS master 验证 CH10 在 FUEL/Diff/精降输出中的打断优先级。

source /opt/ros/noetic/setup.bash
source ~/match_ws/devel/setup.bash
/usr/bin/python3 ~/match_ws/src/autotrans_reference_bridge/test/test_rc_land_priority_ros.py
"""
import os
import signal
import socket
import subprocess
import tempfile
import time
import unittest
import math
from pathlib import Path

import rosgraph
import roslib.packages
import rospy
import rospkg
import yaml
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget, RCIn, State
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu
from quadrotor_msgs.msg import PolynomialTraj, PositionCommand
from std_msgs.msg import Bool, String


class RcLandPriorityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
        os.environ["ROS_HOSTNAME"] = "127.0.0.1"
        cls.log = tempfile.NamedTemporaryFile(prefix="ch10_priority_nodes_", suffix=".log", delete=False)
        print("isolated node log:", cls.log.name)
        cls.master = subprocess.Popen(
            ["roscore", "-p", str(port)], stdout=cls.log, stderr=cls.log,
            start_new_session=True)
        for _ in range(100):
            try:
                rosgraph.Master("/rc_land_test").getPid()
                break
            except Exception:
                time.sleep(0.1)
        else:
            cls.stop(cls.master)
            raise RuntimeError("isolated ROS master did not start")
        rospy.init_node("rc_land_test", anonymous=True, disable_signals=True)

    @classmethod
    def tearDownClass(cls):
        rospy.signal_shutdown("test complete")
        cls.stop(cls.master)
        cls.log.close()

    @staticmethod
    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    def setUp(self):
        self.prefix = "/" + self._testMethodName
        self.processes, self.handles, self.messages = [], [], {}
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for process in reversed(self.processes):
            self.stop(process)
        for handle in self.handles:
            handle.unregister()

    def launch(self, package, executable, name, **parameters):
        node = roslib.packages.find_node(package, executable)[0]
        args = [node, "__name:=" + name, "__ns:=" + self.prefix]
        args.extend("_%s:=%s" % (key, self.prefix + "/" + value)
                    for key, value in parameters.items())
        self.processes.append(subprocess.Popen(
            args, stdout=self.log, stderr=self.log, start_new_session=True))

    def publisher(self, topic, kind, latch=False):
        pub = rospy.Publisher(self.prefix + "/" + topic, kind,
                              queue_size=10, latch=latch)
        self.handles.append(pub)
        return pub

    def subscribe(self, topic, kind):
        self.messages[topic] = []
        sub = rospy.Subscriber(self.prefix + "/" + topic, kind,
                               lambda msg: self.messages[topic].append(msg))
        self.handles.append(sub)

    def wait(self, predicate, message):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if predicate():
                return
            self.assertTrue(all(p.poll() is None for p in self.processes),
                            "test node exited unexpectedly")
            time.sleep(0.03)
        self.fail(message)

    def last(self, topic):
        values = self.messages[topic]
        return values[-1] if values else None

    def run_scenario(self, diff_search):
        self.subscribe("trajectory", PolynomialTraj)
        self.subscribe("trajectory_mux/owner", String)
        self.subscribe("search/state", String)
        self.subscribe("subgoal", PoseStamped)
        self.subscribe("yaw", PositionCommand)
        self.subscribe("trigger", PoseStamped)
        self.subscribe("position", PositionTarget)
        self.subscribe("attitude", AttitudeTarget)
        self.subscribe("owner", String)
        rc_land = self.publisher("rc_land", Bool, latch=True)
        stage = self.publisher("stage", String)
        fuel = self.publisher("fuel", PolynomialTraj)
        diff = self.publisher("diff", PolynomialTraj)
        odom_pub = self.publisher("odom", Odometry)
        need_land = self.publisher("need_land", Bool)
        ctrl_position = self.publisher("controller_position", PositionTarget)
        ctrl_attitude = self.publisher("controller_attitude", AttitudeTarget)
        precision = self.publisher("precision", PositionTarget)
        self.launch("autotrans_reference_bridge", "autotrans_trajectory_arbiter_node",
                    "trajectory_mux", fuel_topic="fuel", diff_topic="diff",
                    output_topic="trajectory", stage_topic="stage",
                    rc_land_active_topic="rc_land")
        self.launch("diff_planner", "landing_diff_search_manager", "search",
                    stage_topic="stage", odom_topic="odom",
                    subgoal_topic="subgoal", yaw_topic="yaw", trigger_topic="trigger",
                    landing_request_topic="landing_request", rc_land_active_topic="rc_land")
        self.launch("precision_landing", "landing_setpoint_arbiter_node", "setpoint_mux",
                    trigger_topic="need_land", controller_position_input="controller_position",
                    controller_attitude_input="controller_attitude",
                    landing_position_input="precision", mavros_position_output="position",
                    mavros_attitude_output="attitude", owner_topic="owner",
                    active_topic="active", rc_land_active_topic="rc_land")
        self.wait(lambda: rc_land.get_num_connections() == 3 and
                  stage.get_num_connections() == 2 and precision.get_num_connections() == 1,
                  "nodes did not connect")
        rc_land.publish(Bool(False))
        odom = Odometry()
        odom.pose.pose.position.z = 1.0
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        if diff_search:
            stage.publish(String("SEARCH_OUTSIDE_LANDING"))
            self.wait(lambda: self.last("trajectory_mux/owner") is not None and
                      self.last("trajectory_mux/owner").data == "DIFF" and
                      self.last("subgoal") is not None, "Diff did not start searching")
        else:
            stage.publish(String("CROSS_EXIT"))
        trajectory = PolynomialTraj(trajectory_id=5, action=PolynomialTraj.ACTION_ADD)
        (diff if diff_search else fuel).publish(trajectory)
        self.wait(lambda: any(m.trajectory_id == 5 and m.action == PolynomialTraj.ACTION_ADD
                             for m in self.messages["trajectory"]),
                  "active planner trajectory was not forwarded")
        # 即使平台精降已取得输出所有权，CH10 也必须能切回 AutoTrans。
        need_land.publish(Bool(True))
        self.wait(lambda: self.last("owner") is not None and
                  self.last("owner").data == "LANDING_PENDING", "precision trigger missing")
        precision.publish(PositionTarget(position=Point(x=9.0)))
        self.wait(lambda: self.last("owner").data == "LANDING", "precision takeover missing")
        rc_land.publish(Bool(True))
        self.wait(lambda: self.last("owner").data == "RC_LAND" and
                  self.last("trajectory_mux/owner").data == "RC_LAND" and
                  self.last("search/state").data == "RC_LAND_ABORTED",
                  "CH10 did not preempt all owners")
        self.assertEqual(self.last("trajectory").action, PolynomialTraj.ACTION_ABORT)
        time.sleep(0.2)
        counts = {key: len(self.messages[key])
                  for key in ("trajectory", "subgoal", "yaw", "trigger")}
        # 延迟消息、反复触发、false 信号均不能恢复旧任务。
        for _ in range(5):
            rc_land.publish(Bool(False))
            stage.publish(String("SEARCH_OUTSIDE_LANDING"))
            stage.publish(String("LANDING"))
            fuel.publish(trajectory)
            diff.publish(trajectory)
            odom_pub.publish(odom)
            need_land.publish(Bool(True))
            precision.publish(PositionTarget(position=Point(x=99.0)))
            ctrl_position.publish(PositionTarget(position=Point(x=3.0)))
            ctrl_attitude.publish(AttitudeTarget(thrust=0.37))
            time.sleep(0.1)
        for key, count in counts.items():
            self.assertEqual(len(self.messages[key]), count, key + " resumed after CH10")
        self.assertEqual(self.last("owner").data, "RC_LAND")
        self.assertEqual(self.last("position").position.x, 3.0)
        self.assertAlmostEqual(self.last("attitude").thrust, 0.37, places=5)
        self.assertEqual(self.last("search/state").data, "RC_LAND_ABORTED")

    def test_cancel_fuel_crossing(self):
        self.run_scenario(False)

    def test_cancel_diff_search(self):
        self.run_scenario(True)

    def test_controller_ch10_locks_xy_yaw_and_descends(self):
        package = "payload_mpc_controller"
        package_path = Path(rospkg.RosPack().get_path(package))
        node_name = self.prefix + "/controller"

        def load_parameters(namespace, values):
            for key, value in values.items():
                path = namespace + "/" + key
                if isinstance(value, dict):
                    load_parameters(path, value)
                else:
                    rospy.set_param(path, value)

        for config in ("mpc.yaml", "model.yaml"):
            load_parameters(node_name, yaml.safe_load((package_path / "config" / config).read_text()))
        self.subscribe("rc_land", Bool)
        self.subscribe("controller/mpc/reference_trajectory", RosPath)
        self.subscribe("attitude", AttitudeTarget)
        odom_pub = self.publisher("odom", Odometry)
        imu_pub = self.publisher("imu", Imu)
        state_pub = self.publisher("state", State)
        rc_pub = self.publisher("rc", RCIn)
        scan_pub = self.publisher("search_state", String)
        yaw_pub = self.publisher("search_yaw", PositionCommand)
        node = roslib.packages.find_node(package, "mpc_controller_node")[0]
        remaps = {"~odom": "odom", "~drone_imu/data": "imu",
                  "/mavros/state": "state", "/mavros/rc/in": "rc",
                  "/mavros/setpoint_raw/attitude": "attitude",
                  "~rc_land_active": "rc_land",
                  "~landing_search_state": "search_state",
                  "~landing_search_yaw": "search_yaw"}
        args = [node, "__name:=controller", "__ns:=" + self.prefix]
        args.extend(key + ":=" + self.prefix + "/" + value for key, value in remaps.items())
        self.processes.append(subprocess.Popen(args, stdout=self.log, stderr=self.log,
                                               start_new_session=True))
        self.wait(lambda: rc_pub.get_num_connections() == 1 and
                  scan_pub.get_num_connections() == 1, "controller did not start")
        odom = Odometry()
        odom.pose.pose.position = Point(1.2, -0.7, 1.5)
        odom.pose.pose.orientation.w = math.cos(0.2)
        odom.pose.pose.orientation.z = math.sin(0.2)
        imu = Imu()
        imu.orientation = odom.pose.pose.orientation
        imu.linear_acceleration.z = 9.795
        state = State(connected=True, armed=True, mode="OFFBOARD")
        rc = RCIn(channels=[1500] * 10)
        rc.channels[7] = 2000
        rc.channels[9] = 1000

        def drive(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                odom.header.stamp = imu.header.stamp = rc.header.stamp = rospy.Time.now()
                odom_pub.publish(odom)
                imu_pub.publish(imu)
                state_pub.publish(state)
                rc_pub.publish(rc)
                time.sleep(0.01)

        drive(1.2)
        # 先进入前视摆头并给与当前朝向不同的航向指令，再用 CH10 打断。
        scan_pub.publish(String("FRONT_ARUCO_YAW_SCAN_LEFT"))
        yaw_pub.publish(PositionCommand(yaw=0.9))
        drive(0.3)
        self.assertIsNotNone(self.last("controller/mpc/reference_trajectory"))
        rc.channels[9] = 2000
        drive(0.35)
        self.assertTrue(self.last("rc_land").data, "controller did not latch accepted CH10")
        yaw_pub.publish(PositionCommand(yaw=-1.0))
        scan_pub.publish(String("FRONT_ARUCO_YAW_SCAN_RIGHT"))
        drive(0.3)
        reference = self.last("controller/mpc/reference_trajectory").poses[0].pose
        self.assertAlmostEqual(reference.position.x, 1.2, places=4)
        self.assertAlmostEqual(reference.position.y, -0.7, places=4)
        self.assertLess(reference.position.z, 1.45)
        actual_yaw = 2 * math.atan2(reference.orientation.z, reference.orientation.w)
        self.assertAlmostEqual(actual_yaw, 0.4, places=4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
