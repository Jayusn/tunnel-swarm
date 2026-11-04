#!/usr/bin/env python3
"""顺序发送 Diff 航点；全部到达后触发精降，不消费检测完成标志。"""
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64, String


def world_waypoints(points, entrance_x, entrance_y, floor_z, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(entrance_x + c*x - s*y, entrance_y + s*x + c*y,
             floor_z + z, dwell) for x, y, z, dwell in points]


class Mission:
    def __init__(self):
        self.lock = threading.RLock()
        self.yaw = float(rospy.get_param('~bridge_yaw'))
        self.frame = 'UAV1/camera_init'
        self.points = world_waypoints(
            rospy.get_param('~waypoints'), float(rospy.get_param('~entrance_x')),
            float(rospy.get_param('~entrance_y')), float(rospy.get_param('~floor_z')), self.yaw)
        self.points.append((float(rospy.get_param('~landing_x')),
                            float(rospy.get_param('~landing_y')),
                            float(rospy.get_param('~landing_approach_z')), 0.0))
        self.radius = float(rospy.get_param('~arrival_distance_m', .15))
        self.speed = float(rospy.get_param('~arrival_speed_mps', .15))
        self.platform_delta = float(rospy.get_param('~landing_platform_delta_z', 0.0))
        self.odom = None
        self.state = State()
        self.reference_z = None
        self.index = -1
        self.arrived_at = None
        self.goal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.landing = rospy.Publisher('need_to_land', Bool, queue_size=1, latch=True)
        self.reference = rospy.Publisher('platform_reference', Float64, queue_size=1, latch=True)
        self.status = rospy.Publisher('status', String, queue_size=1, latch=True)
        self.route = rospy.Publisher('route', Path, queue_size=1, latch=True)
        self.subs = [rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size=1),
                     rospy.Subscriber('state', State, self.state_cb, queue_size=1),
                     rospy.Subscriber('start', PoseStamped, self.start_cb, queue_size=1)]
        self.status.publish(String(data='等待起飞后 CH8 高位开始巡检'))
        self.route.publish(Path(header=self.pose(self.points[0]).header,
                                poses=[self.pose(p) for p in self.points]))
        self.timer = rospy.Timer(rospy.Duration(.05), self.tick)

    def pose(self, point):
        msg = PoseStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = point[:3]
        msg.pose.orientation.z = math.sin(self.yaw/2)
        msg.pose.orientation.w = math.cos(self.yaw/2)
        return msg

    def odom_cb(self, msg):
        with self.lock:
            self.odom = msg
            # 小车行驶期间可更新；进入 OFFBOARD 时锁定同一车面的接地高度。
            if self.reference_z is None and not (self.state.armed and self.state.mode == 'OFFBOARD'):
                self.reference.publish(Float64(data=msg.pose.pose.position.z + self.platform_delta))

    def state_cb(self, msg):
        with self.lock:
            if self.reference_z is None and self.odom is not None and msg.armed and msg.mode == 'OFFBOARD':
                self.reference_z = self.odom.pose.pose.position.z + self.platform_delta
                self.reference.publish(Float64(data=self.reference_z))
            self.state = msg

    def start_cb(self, _):
        with self.lock:
            if self.index != -1:
                return
            self.index = 0
            self.send_goal()

    def send_goal(self):
        self.arrived_at = None
        self.goal.publish(self.pose(self.points[self.index]))
        self.status.publish(String(data='巡检航点 %d/%d' % (self.index + 1, len(self.points))))

    def tick(self, _):
        with self.lock:
            if self.odom is None or not 0 <= self.index < len(self.points):
                return
            p, v = self.odom.pose.pose.position, self.odom.twist.twist.linear
            x, y, z, dwell = self.points[self.index]
            at_point = (math.sqrt((p.x-x)**2+(p.y-y)**2+(p.z-z)**2) <= self.radius and
                        math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z) <= self.speed)
            if not at_point:
                self.arrived_at = None
                return
            now = rospy.Time.now()
            if self.arrived_at is None:
                self.arrived_at = now
            if (now-self.arrived_at).to_sec() < dwell:
                return
            self.index += 1
            if self.index == len(self.points):
                self.status.publish(String(data='航点完成：18 cm ArUco 精降'))
                self.landing.publish(Bool(data=True))
            else:
                self.send_goal()


if __name__ == '__main__':
    rospy.init_node('bridge_mission')
    mission = Mission()
    rospy.spin()
