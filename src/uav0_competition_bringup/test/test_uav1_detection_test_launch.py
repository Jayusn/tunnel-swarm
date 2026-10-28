from pathlib import Path

import roslaunch


ROOT = Path(__file__).resolve().parents[1]


def load(name, argv=None):
    config = roslaunch.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(ROOT / "launch" / name), config,
                                       argv=argv or [], verbose=False)
    return config


def test_standalone_reuses_production_nodes_and_thresholds():
    production = load("uav1_detection_stack.launch")
    standalone = load("uav1_detection_test.launch", ["mission_id:=test_run"])
    test_nodes = {node.name: node for node in standalone.nodes}
    for node in production.nodes:
        same = test_nodes[node.name]
        assert (same.package, same.type, same.remap_args) == (node.package, node.type, node.remap_args)
    allowed_differences = {
        "/uav1_color_tag_detector/wait_for_entry_goal",
        "/uav1_qr_detector/wait_for_entry_goal",
        "/uav1_thermal_detector/wait_for_entry_goal",
        "/uav1_target_reporter/wait_for_entry_goal",
        "/uav1_target_reporter/record_root",
        "/uav1_target_reporter/mission_id",
    }
    differences = {key for key, param in production.params.items()
                   if standalone.params[key].value != param.value}
    assert differences == allowed_differences
    for key in allowed_differences:
        if key.endswith("wait_for_entry_goal"):
            assert production.params[key].value is True
            assert standalone.params[key].value is False
    assert standalone.params["/uav1_target_reporter/mission_id"].value == "test_run"
    assert standalone.params["/uav1_target_reporter/save_unlocalized_evidence"].value is False
    assert standalone.params["/uav1_target_reporter/enable_remote_upload"].value is False


def test_standalone_starts_real_localization_without_flight_control():
    config = load("uav1_detection_test.launch")
    executables = {(node.package, node.type) for node in config.nodes}
    assert ("fast_lio", "fastlio_mapping") in executables
    assert ("livox_ros_driver2", "livox_ros_driver2_node") in executables
    assert ("target_reporting", "publish_camera_extrinsic_tf.py") in executables
    assert ("target_reporting", "uav1_four_view_node.py") in executables
    assert not any(node.package in {"mavros", "control", "payload_mpc_controller", "diff_planner"}
                   for node in config.nodes)
    assert config.params["/UAV1/publish/world_frame"].value == "UAV1/camera_init"
    assert config.params["/uav1_target_reporter/localization_frame"].value == "UAV1/camera_init"


def test_headless_and_no_thermal_options():
    config = load("uav1_detection_test.launch", ["open_rviz:=false", "enable_thermal:=false"])
    assert not any(node.package in {"rviz", "uvc_ubuntu"} for node in config.nodes)
    assert any(node.package == "qr_detector" for node in config.nodes)
    assert any(node.package == "color_tag_detector" for node in config.nodes)
