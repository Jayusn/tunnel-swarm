from pathlib import Path
import xml.etree.ElementTree as ET


LAUNCH = Path(__file__).resolve().parents[1] / "launch/exp/run_swarm_indoor1_fuel_exploration.launch"


def test_landing_search_switches_from_fuel_to_diff_with_separate_commands():
    root = ET.parse(LAUNCH).getroot()
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
    assert manager_params["yaw_topic"] == "/UAV0/landing_diff/yaw"


def test_fuel_declares_external_landing_planner():
    root = ET.parse(LAUNCH).getroot()
    values = {item.attrib.get("param"): (item.text or "").strip() for item in root.findall("rosparam")}
    assert values["/exploration_node/mission/task_search/exit/external_landing_planner"] == "true"
