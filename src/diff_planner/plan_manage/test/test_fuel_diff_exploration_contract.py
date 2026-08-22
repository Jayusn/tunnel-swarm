from pathlib import Path
import xml.etree.ElementTree as ET


LAUNCH = Path(__file__).resolve().parents[1] / "launch/exp/run_swarm_indoor1_fuel_exploration.launch"
ADVANCED = Path(__file__).resolve().parents[1] / "launch/include/advanced_param_exp.xml"


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
