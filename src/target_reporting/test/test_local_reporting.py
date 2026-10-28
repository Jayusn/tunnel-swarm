import importlib.util
import json
from pathlib import Path
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest
import rospy
from geometry_msgs.msg import PointStamped


SCRIPT = Path(__file__).parents[1] / "scripts/target_reporter_node.py"
SPEC = importlib.util.spec_from_file_location("local_reporting_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize("configured_id", ["auto", ""])
def test_uav1_restarts_create_separate_mission_directories(monkeypatch, tmp_path, configured_id):
    params = {"~drone_id": "uav1", "~mission_id": configured_id,
              "~record_root": str(tmp_path), "~enable_remote_upload": False}
    monkeypatch.setattr(MODULE.rospy, "get_param", lambda key, default=None: params.get(key, default))
    for name in ("Publisher", "Subscriber", "Timer", "on_shutdown", "loginfo"):
        monkeypatch.setattr(MODULE.rospy, name, MagicMock())
    monkeypatch.setattr(MODULE.tf2_ros, "Buffer", MagicMock())
    monkeypatch.setattr(MODULE.tf2_ros, "TransformListener", MagicMock())
    times = iter([(2026, 10, 27, 14, 30, 0, 1, 300, -1),
                  (2026, 10, 27, 15, 5, 0, 1, 300, -1)])
    strftime = MODULE.time.strftime
    monkeypatch.setattr(MODULE.time, "strftime", lambda fmt: strftime(fmt, next(times)))

    first = MODULE.TargetReporterNode()
    evidence_path = Path(first.store.save_image("first.jpg", b"existing evidence"))
    first.shutdown()
    second = MODULE.TargetReporterNode()

    assert Path(first.store.mission_dir) == tmp_path / "20261027" / "flight_143000"
    assert Path(second.store.mission_dir) == tmp_path / "20261027" / "flight_150500"
    assert first.store.mission_dir != second.store.mission_dir
    assert evidence_path.read_bytes() == b"existing evidence"
    assert list(Path(second.store.image_dir).rglob("*.jpg")) == []
    assert second.store.all_events() == [] and second.seq == 0
    second.shutdown()


@pytest.mark.parametrize("source", ["color", "qr", "thermal"])
@pytest.mark.parametrize("calibrated", [False, True])
def test_local_evidence_without_network_and_upload_after_reenable(
    monkeypatch, tmp_path, source, calibrated
):
    params = {
        "~enable_remote_upload": False,
        "~record_root": str(tmp_path),
        "~mission_id": "local_test",
        "~localization_frame": "UAV1/camera_init",
        "~thermal_confirm_hits": 1,
        "~thermal_confirmation_duration_s": 0.0,
    }
    if calibrated:
        params["~world_to_channel/rotation"] = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        params["~world_to_channel/translation"] = [10.0, 20.0, 30.0]
    monkeypatch.setattr(
        MODULE.rospy, "get_param", lambda key, default=None: params.get(key, default)
    )
    for name in ("Publisher", "Subscriber", "Timer", "on_shutdown", "loginfo"):
        monkeypatch.setattr(MODULE.rospy, name, MagicMock())
    monkeypatch.setattr(MODULE.rospy, "is_shutdown", lambda: False)
    tf_buffer = MagicMock()
    tf_buffer.transform.side_effect = lambda point, *_args: point
    monkeypatch.setattr(MODULE.tf2_ros, "Buffer", lambda **_kwargs: tf_buffer)
    monkeypatch.setattr(MODULE.tf2_ros, "TransformListener", MagicMock())
    clients = {}
    for name in ("JsonAckClient", "ImageAckClient", "LatestJsonClient"):
        clients[name] = MagicMock()
        monkeypatch.setattr(MODULE, name, clients[name])

    reporter = MODULE.TargetReporterNode()
    reporter._is_leader_thermal_source = MagicMock(return_value=False)
    reporter._is_ground_thermal_source = MagicMock(return_value=False)
    evidence_builder = MagicMock(wraps=MODULE.build_evidence_jpeg)
    monkeypatch.setattr(MODULE, "build_evidence_jpeg", evidence_builder)
    point = PointStamped()
    point.header.frame_id = "camera_optical"
    point.header.stamp = rospy.Time.from_sec(10.0)
    point.point.x, point.point.y, point.point.z = 1.0, 2.0, 3.0
    reporter.latest_point[source] = point
    reporter.point_history[source].append(point)
    reporter.point_version[source] = 1
    reporter.status_version[source] = 1
    reporter.latest_status[source] = (
        {"_detector_confirmable": True, "_detector_stable": True}, 1.0
    )
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    reporter.images.add(source, 10.0, frame)
    if source == "thermal":
        reporter.images.add("thermal_d435", 10.0, frame)
    reporter._try_process(source)

    events = reporter.store.all_events()
    assert len(events) == 1
    assert events[0]["message_type"] == "confirmed_target"
    expected_xyz = (8.0, 21.0, 33.0) if calibrated else (1.0, 2.0, 3.0)
    expected_position = dict(zip(("x", "y", "z"), expected_xyz))
    expected_position.update(frame_id="channel", unit="m")
    assert events[0]["position"] == expected_position
    assert evidence_builder.call_args.args[1]["position"] == expected_position
    world_position = {"x": 1.0, "y": 2.0, "z": 3.0}
    if source == "thermal":
        reporter._is_leader_thermal_source.assert_called_once_with(world_position, 10.0, point)
        reporter._is_ground_thermal_source.assert_called_once_with(world_position)
    expected_images = 2 if source == "thermal" else 1
    images = list(Path(reporter.store.image_dir).rglob("*.jpg"))
    assert len(images) == expected_images
    target_type = {"color": "color_tag", "qr": "qr_code", "thermal": "thermal_source"}[source]
    assert all(path.parent == Path(reporter.store.image_dir) / target_type for path in images)
    assert {p.name for p in Path(reporter.store.image_dir).iterdir()} == {
        "qr_code", "color_tag", "thermal_source"
    }
    assert all(cv2.imread(str(path)) is not None for path in images)
    reporter.observation_pub.publish.assert_called_once()
    local_observation = json.loads(reporter.observation_pub.publish.call_args.args[0].data)
    assert local_observation["position"] == {
        "frame_id": "UAV1/camera_init", "unit": "m", **world_position
    }
    reporter.shutdown()
    for client in clients.values():
        client.assert_not_called()

    # Restarting in local mode must not load existing evidence into network queues.
    restarted = MODULE.TargetReporterNode()
    assert restarted.seq == 1
    restarted.shutdown()
    for client in clients.values():
        client.assert_not_called()

    # Re-enabling keeps the existing upload and pending-record replay behavior.
    params["~enable_remote_upload"] = True
    enabled = MODULE.TargetReporterNode()
    for client in clients.values():
        client.assert_called_once()
        client.return_value.start.assert_called_once()
    clients["JsonAckClient"].return_value.enqueue.assert_called_once_with(events[0])
    assert clients["ImageAckClient"].return_value.enqueue.call_count == expected_images
    # A newly confirmed target uses channel coordinates on both remote paths,
    # while local RViz still receives the lidar-world point.
    enabled.latest_point[source] = point
    enabled.point_history[source].append(point)
    enabled.point_version[source] = 1
    enabled.status_version[source] = 1
    enabled.latest_status[source] = (
        {"_detector_confirmable": True, "_detector_stable": True}, 1.0
    )
    enabled.images.add(source, 10.0, frame)
    if source == "thermal":
        enabled.images.add("thermal_d435", 10.0, frame)
    enabled._try_process(source)
    remote_observation = clients["LatestJsonClient"].return_value.publish.call_args.args[0]
    remote_event = clients["JsonAckClient"].return_value.enqueue.call_args.args[0]
    assert remote_observation["position"] == expected_position
    assert remote_event["position"] == expected_position
    assert json.loads(enabled.observation_pub.publish.call_args.args[0].data)["position"] == {
        "frame_id": "UAV1/camera_init", "unit": "m", **world_position
    }
    enabled.shutdown()
    for client in clients.values():
        client.return_value.stop.assert_called_once()
