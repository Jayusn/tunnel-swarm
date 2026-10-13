#!/usr/bin/env python3
"""Read-only center-offset sampling; never applies calibration to flight config."""
import argparse
from datetime import datetime
import math
from pathlib import Path
import statistics
import threading
import time

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32
import yaml


def summarize(samples, duration):
    if len(samples) < 2:
        raise ValueError("有效样本不足，未生成补偿结果")
    axes = list(zip(*(s[2:] for s in samples)))
    mean = [statistics.mean(axis) for axis in axes]
    std = [statistics.pstdev(axis) for axis in axes]
    gaps = [samples[i][0] - samples[i - 1][0] for i in range(1, len(samples))]
    return {
        "sample_count": len(samples),
        "requested_duration_sec": duration,
        "sample_span_sec": samples[-1][0] - samples[0][0],
        "max_sample_gap_sec": max(gaps),
        "candidate_center_bias_xy_m": mean[:2],
        "stddev_xy_m": std[:2],
        "mean_marker_height_m": -mean[2],
        "note": "仅当物理中心正对码中心且机身水平时，XY均值才可作为中心偏置；不是完整外参，未写入飞行配置。",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--namespace", default="/UAV0/center_offset_observer")
    parser.add_argument("--output")
    args = parser.parse_args(rospy.myargv()[1:])
    if not math.isfinite(args.duration) or args.duration < 1:
        parser.error("duration must be finite and >= 1 second")
    rospy.init_node("sample_center_offset", anonymous=True)
    samples, ids = [], set()
    lock = threading.Lock()
    start = time.monotonic()

    def point_cb(msg):
        values = (msg.point.x, msg.point.y, msg.point.z)
        stamp = msg.header.stamp.to_sec()
        age = (rospy.Time.now() - msg.header.stamp).to_sec()
        if not all(math.isfinite(v) for v in values) or stamp <= 0 or not -0.05 <= age <= 0.3:
            return
        with lock:
            now = time.monotonic()
            if now - start > args.duration or (samples and stamp <= samples[-1][1]):
                return
            samples.append((now - start, stamp, *values))

    def id_cb(msg):
        if msg.data >= 0:
            with lock:
                ids.add(msg.data)

    point_sub = rospy.Subscriber(args.namespace + "/target_body", PointStamped, point_cb, queue_size=1)
    id_sub = rospy.Subscriber(args.namespace + "/locked_id", Int32, id_cb, queue_size=1)
    print("开始采样：保持机身水平，物理中心正对码中心，持续 %.1f 秒" % args.duration, flush=True)
    while not rospy.is_shutdown() and time.monotonic() - start < args.duration:
        time.sleep(0.05)
    point_sub.unregister()
    id_sub.unregister()
    with lock:
        result = summarize(list(samples), args.duration)
        result["observed_marker_ids"] = sorted(ids)
    result["created_at"] = datetime.now().astimezone().isoformat()
    result["topic"] = args.namespace + "/target_body"
    result["quality_review_needed"] = (
        len(ids) != 1 or result["sample_count"] < 20 or
        result["sample_span_sec"] < args.duration * 0.8 or
        result["max_sample_gap_sec"] > 0.3 or max(result["stddev_xy_m"]) > 0.02)
    path = Path(args.output) if args.output else (
        Path.home() / "target_reports" / "center_offset" /
        (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".yaml"))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        yaml.safe_dump(result, handle, allow_unicode=True, sort_keys=False)
    print(yaml.safe_dump(result, allow_unicode=True, sort_keys=False))
    print("已保存：", path)
    if result["quality_review_needed"]:
        print("样本覆盖不足、丢帧或晃动较大，请检查数据后重测；没有应用任何补偿。")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError) as error:
        raise SystemExit(str(error))
