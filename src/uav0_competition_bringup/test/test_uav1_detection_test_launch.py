from pathlib import Path
import shlex

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
    assert ("odom_visualization", "odom_visualization") in executables
    assert not any(node.package in {"mavros", "control", "payload_mpc_controller", "diff_planner"}
                   for node in config.nodes)
    assert config.params["/UAV1/publish/world_frame"].value == "UAV1/camera_init"
    assert config.params["/uav1_target_reporter/localization_frame"].value == "UAV1/camera_init"


def test_standalone_reuses_multivehicle_visualization():
    import rospkg

    production = roslaunch.ROSLaunchConfig()
    launch = Path(rospkg.RosPack().get_path("exploration_control")) / "launch/uav1_relay_visualization.launch"
    roslaunch.xmlloader.XmlLoader().load(str(launch), production, verbose=False)
    standalone = load("uav1_detection_test.launch")
    nodes = {node.name: node for node in standalone.nodes}
    for node in production.nodes:
        actual = nodes[node.name]
        assert (actual.package, actual.type, actual.args, actual.remap_args) == (
            node.package, node.type, node.args, node.remap_args)
    for key, param in production.params.items():
        assert standalone.params[key].value == param.value
    assert sum(node.package == "rviz" for node in standalone.nodes) == 1


def test_headless_and_no_thermal_options():
    config = load("uav1_detection_test.launch", ["open_rviz:=false", "enable_thermal:=false"])
    assert not any(node.package in {"rviz", "uvc_ubuntu"} for node in config.nodes)
    assert any(node.package == "qr_detector" for node in config.nodes)
    assert any(node.package == "color_tag_detector" for node in config.nodes)


def test_recording_uses_session_directory_and_can_be_disabled():
    config = load("uav1_detection_test.launch", [
        "record_root:=/tmp/detection test", "mission_id:=20261028/test_run"])
    recorder = next(n for n in config.nodes if n.name == "uav1_detection_test_recorder")
    args = shlex.split(recorder.args)
    assert args[0] == "/tmp/detection test/20261028/test_run/bags/detection"
    assert {
        "/UAV1/camera/color/image_raw/compressed",
        "/UAV1/camera/aligned_depth_to_color/image_raw/compressedDepth",
        "/UAV1/vision/qr_candidate_detected", "/UAV1/vision/qr_detected",
        "/UAV1/thermal/report_candidate", "/UAV1/thermal/d435_evidence_image",
        "/UAV1/target_reporting/observation", "/tf", "/tf_static",
    }.issubset(args[1:])
    disabled = load("uav1_detection_test.launch", ["record_bag:=false"])
    assert not any(n.name == recorder.name for n in disabled.nodes)
