#!/usr/bin/env python3
"""独立测试专用：起飞与全部精降目标交给NMPC，统一飞控输出。"""
import copy
import math
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget, State
from std_msgs.msg import String


class Router:
    def __init__(self):
        self.lock = threading.RLock()
        self.stage = ''
        self.local_pose = None
        self.landing_started = False
        self.takeoff_z = None
        self.takeoff_stamp = None
        self.requested = False
        self.owned = False
        self.finished = False
        self.flying = False
        self.state_stamp = rospy.Time(0)
        self.position = rospy.Publisher('mavros_position', PositionTarget, queue_size=1)
        self.attitude = rospy.Publisher('mavros_attitude', AttitudeTarget, queue_size=1)
        self.target = rospy.Publisher('nmpc_target', PositionTarget, queue_size=1)
        self.owner = rospy.Publisher('owner', String, queue_size=1, latch=True)
        self.owner.publish(String(data='WAITING_NMPC'))
        self.subs = [
            rospy.Subscriber('local_pose', PoseStamped, self.pose_cb, queue_size=1),
            rospy.Subscriber('takeoff_position', PoseStamped, self.takeoff_cb, queue_size=1),
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
            if self.owned and not self.flying:
                self.finished = True
                self.owner.publish(String(data='STOPPED'))

    def pose_cb(self, msg):
        with self.lock:
            p, q = msg.pose.position, msg.pose.orientation
            if (self.fresh(msg) and all(math.isfinite(x) for x in
                    (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) and
                    .5 < q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1.5):
                self.local_pose = msg

    @staticmethod
    def pose_target(msg):
        target = PositionTarget(coordinate_frame=1, type_mask=2552)
        target.header = copy.deepcopy(msg.header)
        target.position = copy.deepcopy(msg.pose.position)
        q = msg.pose.orientation
        target.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return target

    def takeoff_cb(self, msg):
        with self.lock:
            if (self.finished or self.landing_started or not self.fresh(msg) or
                    self.local_pose is None or not self.fresh(self.local_pose)):
                return
            now = rospy.Time.now()
            # 解锁前仅用当前位置预流以建立OFFBOARD；起飞目标绝不直发PX4。
            hold = self.pose_target(self.local_pose)
            hold.header.stamp = now
            if not self.owned:
                self.position.publish(hold)
            if not self.output_allowed():
                return
            if self.takeoff_z is None:
                self.takeoff_z = self.local_pose.pose.position.z
                self.takeoff_stamp = now
            dt = max(0.0, min(.1, (now-self.takeoff_stamp).to_sec()))
            self.takeoff_stamp = now
            self.takeoff_z = min(msg.pose.position.z, self.takeoff_z + .15*dt)
            target = self.pose_target(msg)
            target.position.z = self.takeoff_z
            self.requested = True
            self.target.publish(target)

    def landing_cb(self, msg):
        with self.lock:
            if (not self.output_allowed() or not self.fresh(msg) or
                    self.stage in ('', 'IDLE', 'DONE')):
                return
            self.landing_started = True
            self.requested = True
            self.target.publish(msg)
            # 尚未得到NMPC输出时只维持当前位置，不用PX4执行视觉速度。
            if not self.owned and self.local_pose is not None and self.fresh(self.local_pose):
                hold = self.pose_target(self.local_pose)
                hold.header.stamp = rospy.Time.now()
                self.position.publish(hold)

    def take_control(self):
        if not self.owned:
            self.owned = True
            self.owner.publish(String(data='NMPC'))
            rospy.logwarn('精降测试：全程NMPC已接管，停止当前位置预流')

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
