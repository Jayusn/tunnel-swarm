#!/usr/bin/env python3
"""顺序发送 Diff 航点；全部到达后触发精降，不消费检测完成标志。"""
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64, String


class Mission:
    def __init__(self):
        self.lock = threading.RLock()
        self.yaw = float(rospy.get_param('~bridge_yaw'))
        self.frame = 'UAV1/camera_init'
        # 航点直接使用 FAST-LIO 地图 XYZ，不叠加入口或底板偏移。
        waypoints = rospy.get_param('~waypoints')
        self.points = [tuple(point[:3]) for point in waypoints]
        self.hold_seconds = [float(point[3]) if len(point) > 3 else 0.0
                             for point in waypoints]
        self.hold_seconds.append(0.0)  # 自动追加的降落接近点不额外等待。
        self.turn_degrees = [float(point[4]) if len(point) > 4 else 0.0
                             for point in waypoints] + [0.0]
        self.returning_from_turn = False
        self.arrived_at = None
        self.landing_yaw_stable_since = None
        self.landing_turn_started = False
        self.points.append((float(rospy.get_param('~landing_x')),
                            float(rospy.get_param('~landing_y')),
                            float(rospy.get_param('~landing_approach_z'))))
        # 最后接近航段与精降指定同一绝对航向，不再由两点连线决定。
        self.point_yaws = [self.yaw] * len(self.points)
        self.point_yaws[-1] = math.radians(float(rospy.get_param('~landing_yaw_deg', -90.0)))
        self.radius = float(rospy.get_param('~arrival_distance_m', .15))
        self.speed = float(rospy.get_param('~arrival_speed_mps', .15))
        self.platform_delta = float(rospy.get_param('~landing_platform_delta_z', 0.0))
        self.odom = None
        self.state = State()
        self.reference_z = None
        self.index = -1
        self.goal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.turn_goal = rospy.Publisher('turn_goal', PoseStamped, queue_size=1)
        self.landing = rospy.Publisher('need_to_land', Bool, queue_size=1, latch=True)
        self.reference = rospy.Publisher('platform_reference', Float64, queue_size=1, latch=True)
        self.status = rospy.Publisher('status', String, queue_size=1, latch=True)
        self.route = rospy.Publisher('route', Path, queue_size=1, latch=True)
        self.subs = [rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size=1),
                     rospy.Subscriber('state', State, self.state_cb, queue_size=1),
                     rospy.Subscriber('start', PoseStamped, self.start_cb, queue_size=1)]
        self.status.publish(String(data='等待起飞后 CH8 高位开始巡检'))
        self.route.publish(Path(header=self.pose(self.points[0]).header,
                                poses=[self.pose(p, yaw) for p, yaw in
                                       zip(self.points, self.point_yaws)]))
        self.timer = rospy.Timer(rospy.Duration(.05), self.tick)

    def pose(self, point, yaw=None):
        if yaw is None:
            yaw = self.yaw
        msg = PoseStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = point[:3]
        msg.pose.orientation.z = math.sin(yaw/2)
        msg.pose.orientation.w = math.cos(yaw/2)
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
        self.returning_from_turn = False
        self.goal.publish(self.pose(self.points[self.index], self.point_yaws[self.index]))
        self.status.publish(String(data='巡检航点 %d/%d' % (self.index + 1, len(self.points))))

    def turn_complete(self):
        base_yaw = self.point_yaws[self.index]
        target_yaw = base_yaw if self.returning_from_turn else (
            base_yaw + math.radians(self.turn_degrees[self.index]))
        # 单独请求悬停转向，不给 Diff 重发同坐标航点；NMPC 仅在轨迹结束后接受。
        self.turn_goal.publish(self.pose(self.points[self.index], target_yaw))
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        error = math.atan2(math.sin(target_yaw-yaw), math.cos(target_yaw-yaw))
        if self.returning_from_turn:
            return abs(error) <= math.radians(5.0)
        now = rospy.Time.now()
        if self.arrived_at is None:
            if abs(error) > math.radians(5.0):
                return False
            # 首次转向到位后只计时一次；悬停波动不重新开始保持时长。
            self.arrived_at = now
            self.status.publish(String(data='航点 %d 转向到位：保持 %.1f 秒后转回' %
                                       (self.index+1, self.hold_seconds[self.index])))
        if (now-self.arrived_at).to_sec() >= self.hold_seconds[self.index]:
            self.returning_from_turn = True
            self.arrived_at = None
            self.status.publish(String(data='航点 %d：转回原航向' % (self.index+1)))
        return False

    def tick(self, _):
        with self.lock:
            if self.odom is None or not 0 <= self.index < len(self.points):
                return
            p, v = self.odom.pose.pose.position, self.odom.twist.twist.linear
            x, y, z = self.points[self.index]
            at_point = (math.sqrt((p.x-x)**2+(p.y-y)**2+(p.z-z)**2) <= self.radius and
                        math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z) <= self.speed)
            hold = self.hold_seconds[self.index]
            if self.index == len(self.points) - 1:
                # 复用到点原地转向接口；精降捕获航向前必须连续摆正。
                if at_point:
                    self.landing_turn_started = True
                if self.landing_turn_started:
                    self.turn_goal.publish(self.pose(self.points[self.index], self.point_yaws[self.index]))
                q = self.odom.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
                error = math.remainder(self.point_yaws[self.index] - yaw, 2*math.pi)
                if not at_point or abs(error) > math.radians(5.0):
                    self.landing_yaw_stable_since = None
                    return
                now = rospy.Time.now()
                if self.landing_yaw_stable_since is None:
                    self.landing_yaw_stable_since = now
                if (now - self.landing_yaw_stable_since).to_sec() < 0.5:
                    return
            elif self.turn_degrees[self.index] != 0.0:
                # 开始保持前仍要求到点；计时开始后持续发转向/转回请求。
                if not at_point and self.arrived_at is None and not self.returning_from_turn:
                    return
                if not self.turn_complete() or not at_point:
                    return
            else:
                if not at_point:
                    return
                if hold > 0.0:
                    now = rospy.Time.now()
                    if self.arrived_at is None:
                        self.arrived_at = now
                        self.status.publish(String(data='航点 %d/%d 到达：保持 %.1f 秒' %
                                                   (self.index + 1, len(self.points), hold)))
                    if (now - self.arrived_at).to_sec() < hold:
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
