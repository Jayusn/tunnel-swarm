from pathlib import Path
import xml.etree.ElementTree as ET


LAUNCH = Path(__file__).resolve().parents[1] / "launch/exp/run_swarm_indoor1_fuel_exploration.launch"
ADVANCED_PARAMS = Path(__file__).resolve().parents[1] / "launch/include/advanced_param_exp.xml"
UAV0_TEST_LAUNCH = (
    Path(__file__).resolve().parents[5]
    / "uav0_competition_bringup/launch/uav0_search_landing_test.launch"
)
TRAJ_CONTAINER = (
    Path(__file__).resolve().parents[2]
    / "traj_utils/include/traj_utils/plan_container.hpp"
)
UAV1_RELAY_LAUNCH = (
    Path(__file__).resolve().parents[1]
    / "launch/exp/run_uav1_relay_diff.launch"
)


def test_landing_search_switches_from_fuel_to_diff_with_separate_commands():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    assert args["fuel_max_vel"] == "0.20"
    assert args["landing_search_max_vel"] == "0.30"
    assert args["landing_search_max_acc"] == "0.60"
    assert args["enable_search_landing"] == "$(optenv UAV0_ENABLE_SEARCH_LANDING true)"
    nodes = {node.attrib.get("name"): node for node in root.findall("node")}
    assert "fuel_traj_server" in nodes
    assert "landing_diff_traj_server" in nodes
    assert "planner_command_arbiter" in nodes
    assert "landing_diff_search_manager" in nodes
    assert nodes["landing_diff_traj_server"].attrib["if"] == "$(arg enable_search_landing)"
    assert nodes["landing_diff_search_manager"].attrib["if"] == "$(arg enable_search_landing)"
    assert "if" not in nodes["fuel_traj_server"].attrib

    fuel_remaps = {item.attrib["from"]: item.attrib["to"] for item in nodes["fuel_traj_server"].findall("remap")}
    diff_remaps = {item.attrib["from"]: item.attrib["to"] for item in nodes["landing_diff_traj_server"].findall("remap")}
    assert fuel_remaps["/position_cmd"] == "/UAV0/fuel/planning/pos_cmd"
    assert diff_remaps["position_cmd"] == "/UAV0/diff/planning/pos_cmd"
    assert diff_remaps["~planning/yaw"] == "/UAV0/landing_diff/yaw"

    manager_params = {item.attrib["name"]: item.attrib["value"] for item in nodes["landing_diff_search_manager"].findall("param")}
    assert manager_params["search_height"] == "2.0"
    assert manager_params["front_approach_distance"] == "1.0"
    assert manager_params["front_approach_arrive_xy"] == "0.15"
    assert manager_params["front_scan_anchor_topic"] == (
        "/UAV0/landing/front_scan_anchor"
    )
    assert manager_params["subgoal_topic"] == "/UAV0/landing_diff/subgoal"
    assert manager_params["marker_topic"] == "/UAV0/mission/detection/final_aruco"
    assert manager_params["front_hint_topic"] == "/UAV0/landing/front_aruco_hint"
    assert manager_params["assigned_target_topic"] == "/UAV0/landing/assigned_target"
    assert manager_params["enable_single_front_hint_approach"] == "false"
    assert manager_params["yaw_topic"] == "/UAV0/landing_diff/yaw"
    assert manager_params["front_hint_dwell_sec"] == "2.0"
    assert manager_params["front_hint_initial_wait_sec"] == "1.0"
    assert manager_params["front_yaw_scan_half_angle_rad"] == "0.7854"
    assert manager_params["front_yaw_scan_rate_rad_s"] == "0.35"
    assert manager_params["front_yaw_arrive_tolerance_rad"] == "0.12"
    assert manager_params["front_yaw_return_dwell_sec"] == "0.50"
    assert manager_params["target_frame"] == "world"
    assert manager_params["marker_timeout_sec"] == "0.50"
    assert manager_params["front_hint_timeout_sec"] == "0.50"
    assert manager_params["assigned_target_timeout_sec"] == "0.50"

    manager_source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    assert "ASSIGNED_TARGET_REACHED_WAIT_DOWN_CONFIRMATION" in manager_source
    assert "DIFF_APPROACH_REACHED_LANDING_REQUESTED" in manager_source
    assert manager_source.index("if (have_marker_)") < manager_source.index(
        "else if (have_assigned_target_)"
    )


def test_fuel_declares_external_landing_planner():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    values = {item.attrib.get("param"): (item.text or "").strip() for item in root.findall("rosparam")}
    params = {item.attrib.get("name"): item for item in root.findall("param")}
    assert values["/exploration_node/mission/task_search/exit/external_landing_planner"] == "true"
    assert values["/exploration_node/mission/task_search/exit/enabled"] == "true"
    assert args["enable_auto_exit"] == "$(optenv UAV0_ENABLE_AUTO_EXIT false)"
    assert params["/exploration_node/mission/task_search/exit/auto_crossing_enabled"].attrib == {
        "name": "/exploration_node/mission/task_search/exit/auto_crossing_enabled",
        "value": "$(arg enable_auto_exit)",
        "type": "bool",
    }
    assert params["/exploration_node/mission/task_search/exit/rc_trigger/enabled"].attrib == {
        "name": "/exploration_node/mission/task_search/exit/rc_trigger/enabled",
        "value": "$(arg enable_search_landing)",
        "type": "bool",
    }
    assert values["/exploration_node/mission/task_search/exit/rc_trigger/topic"] == "/UAV0/mavros/rc/in"
    assert values["/exploration_node/mission/task_search/exit/rc_trigger/channel_index"] == "8"
    assert values["/exploration_node/mission/task_search/exit/rc_trigger/low_pwm"] == "1300"
    assert values["/exploration_node/mission/task_search/exit/rc_trigger/high_pwm"] == "1800"
    assert values["/exploration_node/mission/task_search/exit/rc_trigger/hold_sec"] == "0.5"


def test_follower_frame_alignment_is_not_duplicated_in_planner_launch():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"] for item in root.findall("arg")}
    nodes = {node.attrib.get("name") for node in root.findall("node")}
    assert "follower_offset_x" not in args
    assert "follower_offset_y" not in args
    assert "follower_offset_z" not in args
    assert "world_to_UAV1_camera_init" not in nodes


def test_dynamic_obstacle_detection_is_disabled_by_default():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    assert args["enable_uav0_dynamic_obstacle_detection"] == (
        "$(optenv UAV0_ENABLE_DYNAMIC_OBSTACLE_DETECTION false)"
    )
    assert args["enable_uav1_dynamic_obstacle_detection"] == (
        "$(optenv UAV1_ENABLE_DYNAMIC_OBSTACLE_DETECTION false)"
    )

    arbiter = next(
        node for node in root.findall("node")
        if node.attrib.get("name") == "planner_command_arbiter"
    )
    params = {item.attrib["name"]: item.attrib["value"] for item in arbiter.findall("param")}
    assert params["dynamic_avoidance_enabled"] == (
        "$(arg enable_uav0_dynamic_obstacle_detection)"
    )

    ldop_include = next(
        item for item in root.findall("include")
        if "run_ldop_uav_pair.launch" in item.attrib.get("file", "")
    )
    include_args = {item.attrib["name"]: item.attrib["value"] for item in ldop_include.findall("arg")}
    assert include_args["enable_uav0"] == "$(arg enable_uav0_dynamic_obstacle_detection)"
    assert include_args["enable_uav1"] == "$(arg enable_uav1_dynamic_obstacle_detection)"


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


def test_ch9_moves_one_meter_forward_before_front_scan():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    activate = source.split("void activate()", maxsplit=1)[1].split(
        "void beginFrontScan()", maxsplit=1
    )[0]
    assert "front_approach_distance_ * forward_x_" in activate
    assert "front_approach_distance_ * forward_y_" in activate
    assert 'publishState("FRONT_ARUCO_FORWARD_APPROACH")' in activate
    assert 'publishState("FRONT_ARUCO_INITIAL_WAIT")' not in activate

    timer = source.split("void timerCallback", maxsplit=1)[1].split(
        "bool holdingForInitialFrontSearch", maxsplit=1
    )[0]
    approach = timer.split("if (!front_approach_complete_)", maxsplit=1)[1]
    assert "publishSearchYaw()" in approach
    assert "frontApproachArrived()" in approach
    assert "publishGoalIfDue(front_approach_goal_)" in approach
    assert approach.index("frontApproachArrived()") < approach.index(
        "beginFrontScan()"
    )

    begin = source.split("void beginFrontScan()", maxsplit=1)[1].split(
        "void buildSweepGoals()", maxsplit=1
    )[0]
    assert "publishFrontScanAnchor()" in begin
    assert 'publishState("FRONT_ARUCO_INITIAL_WAIT")' in begin

    for callback_name, callback_end in (
        ("void markerCallback", "void frontHintCallback"),
        ("void frontHintCallback", "void assignedTargetCallback"),
        ("void assignedTargetCallback", "void activate()"),
    ):
        callback = source.split(callback_name, maxsplit=1)[1].split(
            callback_end, maxsplit=1
        )[0]
        assert "!front_approach_complete_" in callback


def test_yaw_scan_waits_for_measured_yaw_before_advancing():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    scan = source.split("bool advanceFrontYawScan()", maxsplit=1)[1].split(
        "bool returnFrontYawToExitHeading()", maxsplit=1
    )[0]
    assert "yawFromQuaternion(odom_.pose.pose.orientation)" in scan
    assert "shortestYawDifference(actual_yaw, target_yaw)" in scan
    assert "!std::isfinite(actual_yaw)" in scan
    assert "actual_error > front_yaw_arrive_tolerance_rad_" in scan
    assert scan.index("actual_error > front_yaw_arrive_tolerance_rad_") < scan.index(
        "++front_yaw_scan_phase_"
    )
    assert 'publishState("FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP")' in scan
    assert scan.index("actual_error > front_yaw_arrive_tolerance_rad_") < scan.index(
        'publishState("FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP")'
    )


def test_yaw_scan_turns_left_then_right_when_two_codes_are_not_ready():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    scan = source.split("bool advanceFrontYawScan()", maxsplit=1)[1].split(
        "bool returnFrontYawToExitHeading()", maxsplit=1
    )[0]
    assert "locked_yaw_ + front_yaw_scan_half_angle_rad_" in scan
    assert "locked_yaw_ - front_yaw_scan_half_angle_rad_" in scan
    assert scan.index("locked_yaw_ + front_yaw_scan_half_angle_rad_") < scan.index(
        "locked_yaw_ - front_yaw_scan_half_angle_rad_"
    )
    commanded = source.split("double commandedYaw() const", maxsplit=1)[1].split(
        "void publishSearchYaw()", maxsplit=1
    )[0]
    assert "if (front_hint_returning_) return front_yaw_scan_yaw_" in commanded
    assert "front_yaw_scan_complete_ ? locked_yaw_ : front_yaw_scan_yaw_" in commanded


def test_two_aruco_assignment_interrupts_scan_but_waits_for_ch9_yaw_return():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    callback = source.split("void assignedTargetCallback", maxsplit=1)[1].split(
        "void activate()", maxsplit=1
    )[0]
    assert "have_assigned_target_ = true" in callback
    assert "!front_yaw_scan_complete_ && !front_hint_returning_" in callback
    assert "if (start_yaw_return)" in callback
    start_return = callback.split("if (start_yaw_return)", maxsplit=1)[1].split(
        "} else {", maxsplit=1
    )[0]
    assert "front_hint_returning_ = true" in start_return
    assert "front_yaw_return_stable_since_ = ros::Time(0)" in start_return
    keepalive = callback.rsplit("} else {", maxsplit=1)[1]
    assert "front_yaw_return_stable_since_ = ros::Time(0)" not in keepalive
    assert "assigned target keepalive updated" in keepalive
    assert "resetting CH9 yaw return dwell" in keepalive
    assert "if (front_yaw_scan_complete_)" in callback
    assert callback.index("have_assigned_target_ = true") < callback.index(
        "if (front_yaw_scan_complete_)"
    )
    assert 'publishState("FRONT_ARUCO_YAW_SCAN_RETURN")' in start_return
    assert (
        'publishState("TWO_ARUCOS_ASSIGNED_APPROACH_FAR_PLATFORM")'
        not in start_return
    )

    scan = source.split("bool advanceFrontYawScan()", maxsplit=1)[1].split(
        "bool returnFrontYawToExitHeading()", maxsplit=1
    )[0]
    completion = scan.split("if (front_yaw_scan_phase_ >= 3)", maxsplit=1)[1]
    assert "front_yaw_scan_complete_ = true" in completion
    assert 'publishState("TWO_ARUCOS_ASSIGNED_APPROACH_FAR_PLATFORM")' in completion
    assert completion.index("front_yaw_scan_complete_ = true") < completion.index(
        'publishState("TWO_ARUCOS_ASSIGNED_APPROACH_FAR_PLATFORM")'
    )
    assert "front_yaw_return_stable_since_" in scan
    assert "front_yaw_return_dwell_sec_" in scan
    assert scan.index("front_yaw_return_dwell_sec_") < scan.index(
        "front_yaw_scan_complete_ = true"
    )

    returning = source.split("bool returnFrontYawToExitHeading()", maxsplit=1)[1].split(
        "void selectNearestSweepGoal()", maxsplit=1
    )[0]
    assert "shortestYawDifference(actual_yaw, locked_yaw_)" in returning
    assert "front_yaw_return_dwell_sec_" in returning
    assert "front_yaw_scan_complete_ = true" in returning
    assert returning.index("front_yaw_scan_complete_ = true") < returning.index(
        'publishState("TWO_ARUCOS_ASSIGNED_APPROACH_FAR_PLATFORM")'
    )


def test_down_sweep_keeps_ch9_locked_yaw_until_landing_handoff():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    commanded = source.split("double commandedYaw() const", maxsplit=1)[1].split(
        "void publishSearchYaw()", maxsplit=1
    )[0]
    assert "front_yaw_scan_complete_" in commanded
    assert "? locked_yaw_" in commanded
    stage = source.split("void stageCallback", maxsplit=1)[1].split(
        "void markerCallback", maxsplit=1
    )[0]
    assert 'publishState("LANDING_HANDOFF")' in stage


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


def test_initial_front_scan_does_not_send_zero_length_diff_goal():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    initial_hold = source.split(
        "if (holdingForInitialFrontSearch())", maxsplit=1
    )[1].split("geometry_msgs::Point goal;", maxsplit=1)[0]
    assert "publishGoal" not in initial_hold


def test_diff_rejects_invalid_and_near_zero_planning_inputs():
    package = Path(__file__).resolve().parents[1]
    fsm_source = (package / "src/diff_replan_fsm.cpp").read_text()
    manager_source = (package / "src/planner_manager.cpp").read_text()
    assert "GOAL_REJECTED_NO_ODOM" in fsm_source
    assert "GOAL_REJECTED_NONFINITE" in fsm_source
    assert "START_REJECTED_OUTSIDE_MAP" in fsm_source
    assert "GOAL_SKIPPED_TOO_CLOSE" in fsm_source
    assert "waypoints.empty()" in manager_source
    assert "segment_distance <= 1.0e-4" in manager_source
    assert "time_vec.allFinite()" in manager_source


def test_search_subgoal_keepalive_is_deduplicated_without_masking_stuck_detection():
    package = Path(__file__).resolve().parents[1]
    source = (package / "src/diff_replan_fsm.cpp").read_text()
    callback = source.split(
        "void DiffReplanFSM::searchSubgoalCallback", maxsplit=1
    )[1].split(
        "void DiffReplanFSM::RecvBroadcastMINCOTrajCallback", maxsplit=1
    )[0]

    assert 'nh.param("fsm/search_subgoal_update_distance"' in source
    assert "if (have_target_)" in callback
    assert "(end_wp - final_goal_).norm()" in callback
    assert "goal_delta < search_subgoal_update_distance_" in callback
    duplicate_branch = callback.split("if (have_target_)", maxsplit=1)[1].split(
        'ROS_INFO("Received search subgoal', maxsplit=1
    )[0]
    assert "return;" in duplicate_branch
    assert "last_target_change_time_" not in duplicate_branch


def test_search_subgoal_dedup_distance_is_explicit_in_uav0_launches():
    advanced_root = ET.parse(ADVANCED_PARAMS).getroot()
    advanced_args = {
        item.attrib["name"]: item.attrib.get("default")
        for item in advanced_root.findall("arg")
    }
    assert advanced_args["search_subgoal_update_distance"] == "0.15"

    for launch_path in (LAUNCH, UAV0_TEST_LAUNCH):
        root = ET.parse(launch_path).getroot()
        diff_include = next(
            item for item in root.findall("include")
            if "advanced_param_exp.xml" in item.attrib.get("file", "")
        )
        include_args = {
            item.attrib["name"]: item.attrib["value"]
            for item in diff_include.findall("arg")
        }
        assert include_args["search_subgoal_update_distance"] == "0.15"


def test_local_trajectory_id_is_not_reset_when_global_goal_changes():
    source = TRAJ_CONTAINER.read_text()
    assert source.count("local_traj.traj_id = 0;") == 1
    set_global = source.split("void setGlobalTraj", maxsplit=1)[1].split(
        "void setLocalTraj", maxsplit=1
    )[0]
    assert "local_traj.traj_id = 0;" not in set_global
    assert "local_traj.traj_id++;" in source


def test_uav1_duplicate_relay_goal_is_acknowledged_without_replanning():
    package = Path(__file__).resolve().parents[1]
    source = (package / "src/diff_replan_fsm.cpp").read_text()
    callback = source.split(
        "void DiffReplanFSM::waypointCallback", maxsplit=1
    )[1].split(
        "void DiffReplanFSM::publishPlanningStatus", maxsplit=1
    )[0]

    assert 'nh.param("fsm/manual_goal_update_distance"' in source
    assert "exec_state_ != WAIT_TARGET" in callback
    assert "exec_state_ != EMERGENCY_STOP" in callback
    assert "(end_wp - final_goal_).norm()" in callback
    assert 'publishPlanningStatus("TRAJECTORY_PUBLISHED")' in callback
    duplicate_branch = callback.split(
        "if (manual_goal_update_distance_ > 0.0", maxsplit=1
    )[1].split('ROS_INFO("Received goal', maxsplit=1)[0]
    assert duplicate_branch.index('publishPlanningStatus("TRAJECTORY_PUBLISHED")') < duplicate_branch.index("return;")
    assert "planNextWaypoint" not in duplicate_branch


def test_uav1_relay_explicitly_enables_manual_goal_deduplication():
    root = ET.parse(UAV1_RELAY_LAUNCH).getroot()
    diff_include = next(
        item for item in root.findall("include")
        if "advanced_param_exp.xml" in item.attrib.get("file", "")
    )
    include_args = {
        item.attrib["name"]: item.attrib["value"]
        for item in diff_include.findall("arg")
    }
    assert include_args["flight_type"] == "1"
    assert include_args["manual_goal_update_distance"] == "0.15"
