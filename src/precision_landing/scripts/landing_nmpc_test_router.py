#!/usr/bin/env python3
"""独立测试专用：锁点前转发PX4目标，收到NMPC输出后切换唯一输出源。"""
import math
import threading
import rospy
from mavros_msgs.msg import AttitudeTarget, PositionTarget, State
from std_msgs.msg import String


class Router:
    def __init__(self):
        self.lock = threading.RLock()
        self.stage = ''
        self.requested = False
        self.owned = False
        self.finished = False
        self.flying = False
        self.state_stamp = rospy.Time(0)
        self.position = rospy.Publisher('mavros_position', PositionTarget, queue_size=1)
        self.attitude = rospy.Publisher('mavros_attitude', AttitudeTarget, queue_size=1)
        self.target = rospy.Publisher('nmpc_target', PositionTarget, queue_size=1)
        self.owner = rospy.Publisher('owner', String, queue_size=1, latch=True)
        self.owner.publish(String(data='PX4'))
        self.subs = [
            rospy.Subscriber('landing_state', String, self.stage_cb, queue_size=1),
            rospy.Subscriber('mavros_state', State, self.state_cb, queue_size=1),
            rospy.Subscriber('landing_position', PositionTarget, self.landing_cb, queue_size=1),
            rospy.Subscriber('nmpc_attitude', AttitudeTarget, self.attitude_cb, queue_size=1),
            rospy.Subscriber('nmpc_recovery', PositionTarget, self.recovery_cb, queue_size=1),
        ]

    @staticmethod
    def fresh(msg):
        age = (rospy.Time.now() - msg.header.stamp).to_sec()
        return not msg.header.stamp.is_zero() and -0.05 <= age <= 0.3

    def output_allowed(self):
        return (not self.finished and self.flying and
                (rospy.Time.now() - self.state_stamp).to_sec() <= 0.5)

    def stage_cb(self, msg):
        with self.lock:
            self.stage = msg.data.split()[0] if msg.data else ''

    def state_cb(self, msg):
        with self.lock:
            self.state_stamp = rospy.Time.now()
            self.flying = msg.connected and msg.armed and msg.mode == 'OFFBOARD'
            if self.requested and not self.flying:
                self.finished = True
                self.owner.publish(String(data='STOPPED'))

    def landing_cb(self, msg):
        with self.lock:
            if not self.output_allowed() or not self.fresh(msg):
                return
            if self.stage in ('FIXED_XY_DESCENT', 'REQUEST_AUTO_LAND'):
                self.requested = True
                self.target.publish(msg)
            if not self.owned:
                self.position.publish(msg)

    def take_control(self):
        if not self.owned:
            self.owned = True
            self.owner.publish(String(data='NMPC'))
            rospy.logwarn('精降测试：NMPC已输出，停止转发精降PX4位置指令')

    def attitude_cb(self, msg):
        with self.lock:
            values = (msg.thrust, msg.body_rate.x, msg.body_rate.y, msg.body_rate.z)
            if (self.requested and self.output_allowed() and self.fresh(msg) and
                    all(math.isfinite(v) for v in values) and 0 <= msg.thrust <= 1):
                self.take_control()
                self.attitude.publish(msg)

    def recovery_cb(self, msg):
        with self.lock:
            if self.requested and self.output_allowed() and self.fresh(msg):
                self.take_control()
                self.position.publish(msg)


if __name__ == '__main__':
    rospy.init_node('landing_nmpc_test_router')
    router = Router()
    rospy.spin()
