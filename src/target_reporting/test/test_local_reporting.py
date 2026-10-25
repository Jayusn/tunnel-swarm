import importlib.util
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


@pytest.mark.parametrize("source", ["color", "qr", "thermal"])
def test_local_evidence_without_network_and_upload_after_reenable(
    monkeypatch, tmp_path, source
):
    params = {
        "~enable_remote_upload": False,
        "~record_root": str(tmp_path),
        "~mission_id": "local_test",
        "~thermal_confirm_hits": 1,
        "~thermal_confirmation_duration_s": 0.0,
    }
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
    expected_images = 2 if source == "thermal" else 1
    images = list(Path(reporter.store.image_dir).glob("*.jpg"))
    assert len(images) == expected_images
    assert all(cv2.imread(str(path)) is not None for path in images)
    reporter.observation_pub.publish.assert_called_once()
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
    enabled.shutdown()
    for client in clients.values():
        client.return_value.stop.assert_called_once()
