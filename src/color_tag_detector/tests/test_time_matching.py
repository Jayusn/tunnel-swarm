from pathlib import Path

import message_filters
import pytest
import rospy
import yaml
from sensor_msgs.msg import Image
from std_msgs.msg import Header


@pytest.mark.parametrize("offset,accepted", [(-.499, True), (.499, True), (-.501, False), (.501, False)])
def test_configured_sync_tolerance_accepts_half_second_range(monkeypatch, offset, accepted):
    monkeypatch.setattr(rospy.Time, "now", staticmethod(lambda: rospy.Time.from_sec(20)))
    config = yaml.safe_load((Path(__file__).parents[1] / "config/color_thresholds.yaml").read_text())
    assert config["sync_slop"] == .50
    color, depth = message_filters.SimpleFilter(), message_filters.SimpleFilter()
    sync = message_filters.ApproximateTimeSynchronizer(
        [color, depth], config["sync_queue_size"], config["sync_slop"])
    matches = []
    sync.registerCallback(lambda a, b: matches.append((a, b)))
    color.signalMessage(Image(header=Header(stamp=rospy.Time.from_sec(10))))
    depth.signalMessage(Image(header=Header(stamp=rospy.Time.from_sec(10 + offset))))
    assert bool(matches) == accepted
