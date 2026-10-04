import pathlib
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]


class PositionRecoveryContract(unittest.TestCase):
    def test_recovery_latches_px4_pose_not_fast_lio_position(self):
        fsm = (ROOT / "src/mpc_fsm.cpp").read_text()
        recovery = fsm.split("void MPCFSM::beginMpcRecovery", 1)[1].split(
            "void MPCFSM::processMpcRecovery", 1)[0]
        self.assertIn("recovery_local_position_ = latest_mavros_local_position_;", recovery)
        self.assertIn("recovery_local_yaw_ = latest_mavros_local_yaw_;", recovery)
        self.assertNotIn("!mpcControlStateValid(now)", recovery)
        self.assertIn("if (!mavros_local_pose_valid_)", recovery)

    def test_recovery_publishes_only_position_target(self):
        fsm = (ROOT / "src/mpc_fsm.cpp").read_text()
        process = fsm.split("void MPCFSM::process()", 1)[1].split(
            "void MPCFSM::CMD_CTRL_process", 1)[0]
        self.assertIn("publish_recovery_position_ctrl(now_time);", process)
        self.assertNotIn("publish_recovery_attitude_ctrl(now_time)", process)
        self.assertNotIn("canReuseLastValidMpc(now_time)", process)
        self.assertNotIn("automatic_state && !odomControlStateValid(now_time)", process)
        publisher = fsm.split("void MPCFSM::publish_recovery_position_ctrl", 1)[1].split(
            "void MPCFSM::publish_recovery_attitude_ctrl", 1)[0]
        for field in ("VX", "VY", "VZ", "AFX", "AFY", "AFZ", "YAW_RATE"):
            self.assertIn("IGNORE_" + field, publisher)
        self.assertNotIn("IGNORE_PX", publisher)
        self.assertNotIn("IGNORE_YAW |", publisher)
        self.assertIn("recovery_position_pub_.publish(msg)", publisher)

    def test_uav0_mavros_topics(self):
        launch = ET.parse(ROOT / "launch/quad_wind_mpc_controller.launch").getroot()
        args = {arg.get("name"): arg.get("default") for arg in launch.findall("arg")}
        self.assertEqual(args["mavros_local_pose_topic"], "/UAV0/mavros/local_position/pose")
        self.assertEqual(args["recovery_position_setpoint_topic"], "/UAV0/mavros/setpoint_raw/local")
        node = launch.find("node")
        remaps = {r.get("from"): r.get("to") for r in node.findall("remap")}
        self.assertEqual(remaps["~mavros_local_pose"], "$(arg mavros_local_pose_topic)")
        self.assertEqual(remaps["~recovery_position_setpoint"], "$(arg recovery_position_setpoint_topic)")


if __name__ == "__main__":
    unittest.main()
