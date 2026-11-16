#!/usr/bin/env python3
"""RRT/MINCO static-scene adapter using match_ws PositionCommand messages."""
import json
import math
from pathlib import Path
import sys
import threading

import numpy as np
import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String

PACKAGE = Path(rospkg.RosPack().get_path('rrt_minco_adapter'))
sys.path.insert(0, str(PACKAGE / 'scripts'))
from plan_rrt import solve


class RrtMincoNode:
    def __init__(self):
        self.scene = json.loads(Path(rospy.get_param('~scene_file')).expanduser().read_text())
        self.frame = rospy.get_param('~frame_id', 'UAV0/camera_init')
        self.options = dict(speed=float(rospy.get_param('~speed', .6)),
                            acceleration=float(rospy.get_param('~acceleration', .8)),
                            clearance=float(rospy.get_param('~clearance', .3)),
                            reserve=float(rospy.get_param('~tracking_margin', .1)))
        for key, value in self.options.items():
            if not math.isfinite(value) or value < 0 or (key != 'reserve' and value == 0):
                raise ValueError('Invalid planning parameter: ' + key)
        bounds = np.asarray(self.scene['bounds'], dtype=float)
        boxes = np.asarray(self.scene['boxes'], dtype=float).reshape(-1, 4)
        if (bounds.shape != (4,) or not np.isfinite(bounds).all()
                or bounds[0] >= bounds[1] or bounds[2] >= bounds[3]
                or not np.isfinite(boxes).all() or np.any(boxes[:, 2:] <= 0)):
            raise ValueError('Invalid static scene bounds/boxes')
        self.lock = threading.Lock()
        self.odom = None
        self.busy = False
        self.active = None
        self.trajectory_id = 0
        self.polynomial_pub = rospy.Publisher('~polynomial', String, queue_size=1)
        self.path_pub = rospy.Publisher('~path', RosPath, queue_size=1, latch=True)
        self.command_pub = rospy.Publisher('~pos_cmd', PositionCommand, queue_size=1)
        self.odom_sub = rospy.Subscriber('~odom', Odometry, self.on_odom, queue_size=1)
        self.goal_sub = rospy.Subscriber('~goal', PoseStamped, self.on_goal, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.02), self.on_timer)
        rospy.loginfo('RRT/MINCO ready: static scene, fixed height, frame=%s', self.frame)

    def on_odom(self, msg):
        with self.lock:
            self.odom = msg

    def fail(self, request, error):
        self.polynomial_pub.publish(String(data=json.dumps(
            dict(order=0, start_ns=request, frame_id=self.frame, error=str(error)))))
        rospy.logwarn('RRT/MINCO request rejected: %s', error)

    def on_goal(self, msg):
        request = str((msg.header.stamp if msg.header.stamp != rospy.Time()
                       else rospy.Time.now()).to_nsec())
        with self.lock:
            odom = self.odom
            if self.busy:
                self.fail(request, 'Planner busy; retry after the current result')
                return
            if odom is None:
                self.fail(request, 'Odometry required')
                return
            age = (rospy.Time.now() - odom.header.stamp).to_sec()
            if odom.header.frame_id != self.frame or msg.header.frame_id != self.frame:
                self.fail(request, 'Odometry, goal and scene must use frame ' + self.frame)
                return
            if age < 0 or age > 1.:
                self.fail(request, 'Odometry timestamp is not fresh')
                return
            p, v, q = odom.pose.pose.position, odom.twist.twist.linear, odom.pose.pose.orientation
            g = msg.pose.position
            values = [p.x, p.y, p.z, v.x, v.y, v.z, g.x, g.y, g.z, q.x, q.y, q.z, q.w]
            if not np.isfinite(values).all():
                self.fail(request, 'Non-finite odometry or goal')
                return
            if np.linalg.norm([v.x, v.y, v.z]) > .15 or abs(p.z - g.z) > .02:
                self.fail(request, 'Stationary fixed-height request required')
                return
            norm = np.linalg.norm([q.x, q.y, q.z, q.w])
            if norm < 1e-6:
                self.fail(request, 'Valid odometry orientation required')
                return
            x, y, z, w = np.array([q.x, q.y, q.z, q.w]) / norm
            yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
            self.busy = True
        threading.Thread(target=self.plan, args=(msg, odom, request, yaw), daemon=True).start()

    def plan(self, goal, odom, request, yaw):
        try:
            p, g = odom.pose.pose.position, goal.pose.position
            trajectory, result = solve(self.scene, [p.x, p.y], [g.x, g.y], **self.options)
            path = RosPath()
            path.header.frame_id = self.frame
            for t in np.linspace(0., trajectory.total_time,
                                 max(2, int(math.ceil(trajectory.total_time / .1)) + 1)):
                xy, _ = trajectory.sample(float(t))
                pose = PoseStamped()
                pose.header.frame_id = self.frame
                pose.pose.position.x, pose.pose.position.y = map(float, xy)
                pose.pose.position.z = g.z
                pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.orientation.w = math.cos(yaw / 2)
                path.poses.append(pose)
            with self.lock:
                start_time = rospy.Time.now()
                self.trajectory_id += 1
                result.update(frame_id=self.frame, dimensions=3, start_ns=request,
                              execution_start_ns=str(start_time.to_nsec()),
                              trajectory_id=self.trajectory_id, yaw=float(yaw),
                              coef_z=[c for _ in trajectory.durations
                                      for c in [0., 0., 0., 0., 0., float(g.z)]])
                path.header.stamp = start_time
                for pose in path.poses:
                    pose.header.stamp = start_time
                self.polynomial_pub.publish(String(data=json.dumps(result, allow_nan=False)))
                self.path_pub.publish(path)
                self.active = (trajectory, start_time, g.z, yaw, self.trajectory_id)
            rospy.loginfo('RRT/MINCO trajectory %d ready: %.3fs planning, %.3fs duration',
                          self.trajectory_id, result['total_wall_s'], trajectory.total_time)
        except Exception as error:
            self.fail(request, error)
        finally:
            with self.lock:
                self.busy = False

    def on_timer(self, _event):
        with self.lock:
            if self.active is None:
                return
            trajectory, start, height, yaw, trajectory_id = self.active
            now = rospy.Time.now()
            elapsed = max(0., (now - start).to_sec())
            t = min(elapsed, trajectory.total_time)
            position, velocity = trajectory.sample(t)
            acceleration = trajectory.derivative(t, 2)
            jerk = trajectory.derivative(t, 3)
            if elapsed >= trajectory.total_time:
                velocity = acceleration = jerk = np.zeros(2)
            msg = PositionCommand()
            msg.header.stamp, msg.header.frame_id = now, self.frame
            msg.position.x, msg.position.y = map(float, position)
            msg.position.z = height
            msg.velocity.x, msg.velocity.y = map(float, velocity)
            msg.acceleration.x, msg.acceleration.y = map(float, acceleration)
            msg.jerk.x, msg.jerk.y = map(float, jerk)
            msg.yaw, msg.yaw_dot = yaw, 0.
            msg.trajectory_id = trajectory_id
            msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            self.command_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('rrt_minco_adapter')
    node = RrtMincoNode()
    rospy.spin()
