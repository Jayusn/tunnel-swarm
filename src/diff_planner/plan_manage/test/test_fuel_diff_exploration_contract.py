from pathlib import Path
import xml.etree.ElementTree as ET


LAUNCH = Path(__file__).resolve().parents[1] / "launch/exp/run_swarm_indoor1_fuel_diff_exploration.launch"
ADVANCED = Path(__file__).resolve().parents[1] / "launch/include/advanced_param_exp.xml"
FUEL_LAUNCH = (
    Path(__file__).resolve().parents[1]
    / "launch/exp/run_swarm_indoor1_fuel_exploration.launch"
)


def test_hybrid_launch_routes_fuel_viewpoints_to_diff_and_uses_ten_cm_map():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib.get("default") for item in root.findall("arg")}
    assert args["use_diff_for_fuel_exploration"] == "true"
    assert args["fuel_diff_goal_topic"] == "/UAV0/fuel_diff/exploration_goal"

    nodes = {node.attrib.get("name"): node for node in root.findall("node")}
    assert "if" not in nodes["landing_diff_traj_server"].attrib
    assert nodes["fuel_traj_server"].attrib["if"] == (
        "$(eval not arg('use_diff_for_fuel_exploration'))"
    )
    arbiter_params = {
        item.attrib["name"]: item.attrib["value"]
        for item in nodes["planner_command_arbiter"].findall("param")
    }
    assert arbiter_params["exploration_goal_topic"] == "$(arg fuel_diff_goal_topic)"

    diff_include = next(
        item for item in root.findall("include")
        if "advanced_param_exp.xml" in item.attrib.get("file", "")
    )
    diff_args = {item.attrib["name"]: item.attrib["value"] for item in diff_include.findall("arg")}
    assert diff_args["map_resolution"] == "0.10"
    assert diff_args["exploration_goal_topic"] == "$(arg fuel_diff_goal_topic)"


def test_advanced_diff_include_exposes_structured_exploration_protocol():
    root = ET.parse(ADVANCED).getroot()
    names = {item.attrib["name"] for item in root.findall("arg")}
    assert {
        "exploration_goal_topic",
        "exploration_status_topic",
        "exploration_cancel_topic",
        "exploration_trigger_topic",
        "mission_stage_topic",
    }.issubset(names)


def test_diff_trajectory_server_holds_endpoint_after_a_viewpoint():
    source = (Path(__file__).resolve().parents[1] / "src/traj_server.cpp").read_text()
    assert "hold_after_trajectory_" in source
    assert "Keep publishing a zero-velocity command" in source


def test_fuel_turn_and_ground_recovery_contract():
    root = ET.parse(FUEL_LAUNCH).getroot()
    params = {
        item.attrib["param"]: (item.text or "").strip()
        for item in root.findall("rosparam")
    }
    assert params[
        "/exploration_node/mission/task_search/recovery/turn_in_place_max_stationary_speed"
    ] == "0.20"
    assert params[
        "/exploration_node/mission/task_search/recovery/turn_in_place_tracking_grace_time"
    ] == "1.0"
    assert params["/exploration_node/mission/ground_ascent/enabled"] == "true"
    assert params["/exploration_node/mission/ground_ascent/cruise_height"] == "0.60"
    assert params[
        "/exploration_node/mission/ground_ascent/min_lower_obstacles"
    ] == "2"


def test_fuel_launch_records_lightweight_planner_diagnostics_on_start():
    root = ET.parse(FUEL_LAUNCH).getroot()
    args = {
        item.attrib["name"]: item.attrib.get("default")
        for item in root.findall("arg")
    }
    assert args["enable_planner_rosbag"] == "true"
    assert args["planner_rosbag_prefix"] == "fuel_planner_$(arg vehicle_ns)"
    assert args["planner_rosbag_split_size_mb"] == "1024"

    recorder = next(
        node for node in root.findall("node")
        if node.attrib.get("name") == "fuel_planner_recorder"
    )
    assert recorder.attrib["pkg"] == "rosbag"
    assert recorder.attrib["type"] == "record"
    assert recorder.attrib["if"] == "$(arg enable_planner_rosbag)"
    assert recorder.attrib["cwd"] == "ROS_HOME"

    record_args = recorder.attrib["args"]
    for option in (
        "--lz4",
        "--split",
        "--size=$(arg planner_rosbag_split_size_mb)",
        "-o $(arg planner_rosbag_prefix)",
    ):
        assert option in record_args
    for topic in (
        "$(arg odom_topic)",
        "/UAV0/planning/bspline",
        "/UAV0/planning/pos_cmd",
        "/UAV0/planning/emergency_brake",
        "/UAV0/planning/safety_hold",
        "/UAV0/sdf_map/occupancy_local_inflate",
        "/UAV0/planning_vis/frontier",
        "/UAV0/planning_vis/trajectory",
        "/UAV0/mission/task_status",
        "/tf_static",
        "/rosout",
    ):
        assert topic in record_args

    for excluded in (
        "cloud_registered",
        "livox/lidar",
        "image_raw",
        "mpc/reference_trajectory",
    ):
        assert excluded not in record_args
