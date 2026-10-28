import json
from pathlib import Path

import pytest
import rospy
from geometry_msgs.msg import PoseStamped

from test_exit_plane_filter import reporter, feed_target
from test_unlocalized_evidence import send_image, send_status, status


def entry_goal(node, monkeypatch, seconds=20):
    monkeypatch.setattr(rospy.Time, "now", staticmethod(lambda: rospy.Time.from_sec(seconds)))
    node._entry_goal_cb(PoseStamped())


@pytest.mark.parametrize("source", ["qr", "color", "thermal"])
def test_waiting_observations_do_not_save_until_entry_goal(reporter, monkeypatch, source):
    reporter.wait_for_entry_goal = True
    for seconds in (10, 11, 12):
        feed_target(reporter, source, .5, seconds=seconds)
        observation = json.loads(reporter.observation_pub.publish.call_args.args[0].data)
        assert observation["candidate"] and not observation["confirmed"]
        assert observation["detector_stable"]
        assert observation["hits"] == 0
    assert reporter.store.all_events() == []
    assert list(Path(reporter.store.image_dir).rglob("*.jpg")) == []
    assert reporter.seq == 0

    entry_goal(reporter, monkeypatch)
    # A delayed pre-entry result/image must not become a saved post-entry event.
    feed_target(reporter, source, .5, seconds=19)
    assert reporter.store.all_events() == []
    feed_target(reporter, source, .5, seconds=21)
    events = reporter.store.all_events()
    assert len(events) == 1 and events[0]["timestamp"] == 21
    assert list(Path(reporter.store.image_dir).rglob("*.jpg"))
    observation = json.loads(reporter.observation_pub.publish.call_args.args[0].data)
    assert observation["confirmed"]


@pytest.mark.parametrize("source", ["qr", "color", "thermal"])
def test_unlocalized_evidence_does_not_queue_ground_images_for_later_save(reporter, monkeypatch, source):
    reporter.wait_for_entry_goal = True
    reporter.save_unlocalized_evidence = True
    send_image(reporter, source, 10)
    send_status(reporter, source, status(source, stamp=10))
    assert not reporter.pending_unlocalized[source]
    assert reporter.store.unlocalized_events() == []
    entry_goal(reporter, monkeypatch)
    send_image(reporter, source, 19)
    send_status(reporter, source, status(source, stamp=19))
    assert reporter.store.unlocalized_events() == []
    send_status(reporter, source, status(source, stamp=21))
    send_image(reporter, source, 21)
    events = reporter.store.unlocalized_events()
    assert len(events) == 1 and events[0]["timestamp"] == 21


def test_entry_goal_unlocks_once_and_disabled_setting_allows_ground_saving(reporter, monkeypatch):
    reporter.wait_for_entry_goal = True
    assert not reporter._recording_allowed(20)
    entry_goal(reporter, monkeypatch)
    entry_goal(reporter, monkeypatch, seconds=25)
    assert reporter.recording_start_stamp == 20
    assert reporter._recording_allowed(21)
    assert not reporter._recording_allowed(19)
    reporter.wait_for_entry_goal = False
    feed_target(reporter, "color", .5, seconds=10)
    assert len(reporter.store.all_events()) == 1
