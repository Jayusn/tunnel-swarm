from pathlib import Path
import xml.etree.ElementTree as ET


LAUNCH = Path(__file__).resolve().parents[1] / "launch/exp/run_swarm_indoor1_fuel_exploration.launch"


def test_landing_search_switches_from_fuel_to_diff_with_separate_commands():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    assert args["fuel_max_vel"] == "0.20"
    assert args["landing_search_max_vel"] == "0.50"
    assert args["landing_search_max_acc"] == "0.60"
    nodes = {node.attrib.get("name"): node for node in root.findall("node")}
    assert "fuel_traj_server" in nodes
    assert "landing_diff_traj_server" in nodes
    assert "planner_command_arbiter" in nodes
    assert "landing_diff_search_manager" in nodes

    fuel_remaps = {item.attrib["from"]: item.attrib["to"] for item in nodes["fuel_traj_server"].findall("remap")}
    diff_remaps = {item.attrib["from"]: item.attrib["to"] for item in nodes["landing_diff_traj_server"].findall("remap")}
    assert fuel_remaps["/position_cmd"] == "/UAV0/fuel/planning/pos_cmd"
    assert diff_remaps["position_cmd"] == "/UAV0/diff/planning/pos_cmd"
    assert diff_remaps["~planning/yaw"] == "/UAV0/landing_diff/yaw"

    manager_params = {item.attrib["name"]: item.attrib["value"] for item in nodes["landing_diff_search_manager"].findall("param")}
    assert manager_params["search_height"] == "2.0"
    assert manager_params["subgoal_topic"] == "/UAV0/landing_diff/subgoal"
    assert manager_params["marker_topic"] == "/UAV0/mission/detection/final_aruco"
    assert manager_params["front_hint_topic"] == "/UAV0/landing/front_aruco_hint"
    assert manager_params["yaw_topic"] == "/UAV0/landing_diff/yaw"
    assert manager_params["front_hint_dwell_sec"] == "2.0"
    assert manager_params["front_hint_initial_wait_sec"] == "1.0"
    assert manager_params["front_yaw_scan_half_angle_rad"] == "0.5236"
    assert manager_params["front_yaw_scan_rate_rad_s"] == "0.35"
    assert manager_params["target_frame"] == "world"
    assert manager_params["marker_timeout_sec"] == "0.50"
    assert manager_params["front_hint_timeout_sec"] == "0.50"


def test_fuel_declares_external_landing_planner():
    root = ET.parse(LAUNCH).getroot()
    values = {item.attrib.get("param"): (item.text or "").strip() for item in root.findall("rosparam")}
    assert values["/exploration_node/mission/task_search/exit/external_landing_planner"] == "true"


def test_landing_targets_require_fresh_world_frame_and_finite_position():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    assert "target.header.frame_id != target_frame_" in source
    assert "target.header.stamp.isZero()" in source
    assert "age_sec > timeout_sec" in source
    assert "!std::isfinite(point.x)" in source


def test_yaw_scan_starts_from_the_exit_locked_yaw():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    activate = source.split("void activate()", maxsplit=1)[1].split(
        "void buildSweepGoals()", maxsplit=1
    )[0]
    assert activate.index("locked_yaw_ = yaw") < activate.index(
        "front_yaw_scan_yaw_ = locked_yaw_"
    )


def test_validated_front_hint_is_latched_while_approaching():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    expiry = source.split("void expireTargets()", maxsplit=1)[1].split(
        "void publishGoal", maxsplit=1
    )[0]
    assert 'validTarget(marker_, marker_timeout_sec_, "final ArUco")' in expiry
    assert "validTarget(front_hint_" not in expiry
