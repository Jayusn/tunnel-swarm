#!/usr/bin/env python3
"""只读等待定位初始化收敛，避免控制器过早锁定坐标偏移。"""
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry


def main():
    rospy.init_node('rrt_wait_localization', anonymous=True, disable_signals=True)
    latest = {}
    lock = threading.Lock()

    def receive(msg, key):
        with lock:
            latest[key] = (msg, time.monotonic())

    subscribers = [rospy.Subscriber(topic, kind, receive, callback_args=key, queue_size=1)
                   for key, topic, kind in [
                       ('odom', '/UAV0/fast_lio/Odom_high_freq', Odometry),
                       ('vision', '/UAV0/mavros/vision_pose/pose', PoseStamped),
                       ('pose', '/UAV0/mavros/local_position/pose', PoseStamped),
                       ('state', '/UAV0/mavros/state', State)]]
    deadline = time.monotonic() + 90.
    since, anchor = None, None
    rospy.loginfo('等待定位连续稳定5秒（请保持飞机静止）；最长等待90秒')
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        now = time.monotonic()
        with lock:
            data = dict(latest)
        ready = all(k in data and now - data[k][1] < (3. if k == 'state' else 1.)
                    for k in ('odom', 'vision', 'pose', 'state'))
        values = None
        if ready and data['state'][0].connected:
            stamp = rospy.Time.now().to_sec()
            ready = all(0. <= stamp - data[k][0].header.stamp.to_sec() < 1.
                        for k in ('odom', 'vision', 'pose'))
            points = [data['odom'][0].pose.pose.position,
                      data['vision'][0].pose.position, data['pose'][0].pose.position]
            values = tuple(v for p in points for v in (p.x, p.y, p.z))
            ready = ready and all(math.isfinite(v) for v in values)
            # 比较坐标差的稳定性，允许两坐标系存在固定原点偏移。
            values += tuple(values[6+i] - values[i] for i in range(3))
        else:
            ready = False
        if not ready:
            since, anchor = None, None
        elif anchor is None or any(abs(a-b) > .05 for a, b in zip(values, anchor)):
            since, anchor = now, values
        elif now - since >= 5.:
            rospy.loginfo('定位已连续稳定5秒，允许启动控制器；MAVROS-FAST-LIO偏移=(%.3f, %.3f, %.3f)',
                          *values[-3:])
            return 0
        rospy.loginfo_throttle(5., '定位尚未稳定：等待新鲜的定位/位姿回传及MAVROS连接，稳定期间各轴变化须≤0.05m')
        time.sleep(.05)
    rospy.logerr('定位等待超时或被中断，未启动控制器。请检查定位链路后重新启动。')
    return 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
