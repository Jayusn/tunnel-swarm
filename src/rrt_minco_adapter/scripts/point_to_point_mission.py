#!/usr/bin/env python3
"""One RRT flight after manual OFFBOARD takeoff, followed by PX4 AUTO.LAND."""
import json
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState, RCIn
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool


class Mission:
    def __init__(self):
        self.frame = rospy.get_param('~frame_id', 'UAV0/camera_init')
        self.start = tuple(rospy.get_param('~start', [.01, .01, .5]))
        self.goal = tuple(rospy.get_param('~goal', [3., 3., .5]))
        self.start_wait = float(rospy.get_param('~start_stable_seconds', 0.))
        self.goal_wait = float(rospy.get_param('~goal_hover_seconds', 2.))
        self.xy_tolerance = float(rospy.get_param('~xy_tolerance', .1))
        self.z_tolerance = float(rospy.get_param('~z_tolerance', .05))
        self.speed_tolerance = float(rospy.get_param('~speed_tolerance', .1))
        values = self.start + self.goal + (self.start_wait, self.goal_wait,
                                          self.xy_tolerance, self.z_tolerance, self.speed_tolerance)
        if (len(self.start) != 3 or len(self.goal) != 3 or not all(map(math.isfinite, values))
                or self.start_wait < 0 or min(values[7:]) <= 0 or self.start[2] != self.goal[2]):
            raise ValueError('Invalid fixed-height mission parameters')
        self.lock = threading.RLock()
        self.odom = None
        self.state = None
        self.state_received = rospy.Time()
        self.extended = None
        self.extended_received = rospy.Time()
        self.require_mpc_rc = rospy.get_param("~require_mpc_rc", False)
        self.rc = None
        self.rc_received = rospy.Time()
        self.map_ready = False
        self.phase = 'WAIT_START'
        self.stable_since = None
        self.request_id = None
        self.trajectory_end = None
        self.status = rospy.Publisher('~status', String, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher('~goal', PoseStamped, queue_size=1)
        self.mode_client = rospy.ServiceProxy('~set_mode', SetMode)
        self.set_phase("WAIT_START")
        self.subscribers = [
            rospy.Subscriber('~odom', Odometry, self.on_odom, queue_size=1),
            rospy.Subscriber('~state', State, self.on_state, queue_size=1),
            rospy.Subscriber('~extended_state', ExtendedState, self.on_extended, queue_size=1),
            rospy.Subscriber('~scene', String, self.on_scene, queue_size=1),
            rospy.Subscriber('~polynomial', String, self.on_result, queue_size=1)]
        self.subscribers.extend([
            rospy.Subscriber('/UAV0/mavros/rc/in', RCIn, self.on_rc, queue_size=1),
            rospy.Subscriber('/UAV0/mission/rc_land_active', Bool, self.on_rc_land, queue_size=1)])
        self.timer = rospy.Timer(rospy.Duration(.05), self.tick)

    def on_rc(self, msg):
        with self.lock:
            self.rc, self.rc_received = msg, rospy.Time.now()

    def on_rc_land(self, msg):
        if msg.data:
            with self.lock:
                self.set_phase('CANCELLED', 'AutoTrans CH10 landing owns control; restart after landing')

    def mpc_command_ready(self, now):
        if not self.require_mpc_rc:
            return True
        return (self.rc is not None and self.fresh(now, self.rc_received, .5)
                and len(self.rc.channels) >= 10 and 1800 < self.rc.channels[7] <= 2200)

    def set_phase(self, phase, detail=''):
        self.phase = phase
        self.stable_since = None
        self.status.publish(String(data=json.dumps(dict(phase=phase, detail=detail))))
        rospy.loginfo('RRT mission: %s %s', phase, detail)

    def on_odom(self, msg):
        with self.lock:
            self.odom = msg

    def on_state(self, msg):
        with self.lock:
            self.state, self.state_received = msg, rospy.Time.now()

    def on_extended(self, msg):
        with self.lock:
            self.extended, self.extended_received = msg, rospy.Time.now()

    def on_scene(self, msg):
        try:
            data = json.loads(msg.data)
            ready = data['frame_id'] == self.frame and 'bounds' in data and 'boxes' in data
        except (ValueError, KeyError, TypeError):
            ready = False
        with self.lock:
            self.map_ready = ready

    def on_result(self, msg):
        with self.lock:
            if self.phase != 'PLANNING':
                return
            try:
                data = json.loads(msg.data)
                if data.get('start_ns') != self.request_id:
                    return
                if data['order'] == 0:
                    self.set_phase('FAILED', data.get('error', 'Planning failed'))
                    return
                durations = data['duration']
                if not durations or not all(math.isfinite(v) and v > 0 for v in durations):
                    raise ValueError('Invalid trajectory duration')
                self.trajectory_end = (rospy.Time.from_sec(int(data['execution_start_ns']) / 1e9)
                                       + rospy.Duration(sum(durations)))
                self.set_phase('FLYING')
            except (ValueError, TypeError, KeyError) as error:
                self.set_phase('FAILED', str(error))

    @staticmethod
    def fresh(now, stamp, timeout=1.):
        return 0 <= (now - stamp).to_sec() <= timeout

    def at_rest(self, target, now):
        if (self.odom is None or self.odom.header.frame_id != self.frame
                or not self.fresh(now, self.odom.header.stamp)):
            return False
        p, v = self.odom.pose.pose.position, self.odom.twist.twist.linear
        values = (p.x, p.y, p.z, v.x, v.y, v.z)
        return (all(map(math.isfinite, values))
                and math.hypot(p.x-target[0], p.y-target[1]) <= self.xy_tolerance
                and abs(p.z-target[2]) <= self.z_tolerance
                and math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z) <= self.speed_tolerance)

    def stable_for(self, condition, seconds, now):
        if not condition:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return (now-self.stable_since).to_sec() >= seconds

    def tick(self, _event):
        with self.lock:
            now = rospy.Time.now()
            if self.phase in ('DONE', 'FAILED', 'CANCELLED'):
                return
            state_fresh = self.state is not None and self.fresh(now, self.state_received, 3.)
            offboard = (state_fresh and self.state.connected and self.state.armed
                        and self.state.mode == 'OFFBOARD')
            if self.phase in ('LAND_REQUESTED', 'LANDING'):
                if state_fresh and self.state.mode == 'AUTO.LAND' and self.phase != 'LANDING':
                    self.set_phase('LANDING')
                if (self.phase == 'LANDING' and state_fresh and not self.state.armed
                        and self.extended is not None and self.fresh(now, self.extended_received, 3.)
                        and self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND):
                    self.set_phase('DONE')
                return
            if self.phase == 'WAIT_START':
                ready = (offboard and self.map_ready and self.mpc_command_ready(now)
                         and self.goal_pub.get_num_connections() > 0)
                if self.stable_for(ready and self.at_rest(self.start, now), self.start_wait, now):
                    msg = PoseStamped()
                    msg.header.stamp, msg.header.frame_id = now, self.frame
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.goal
                    msg.pose.orientation.w = 1.
                    self.request_id = str(now.to_nsec())
                    self.set_phase('PLANNING')
                    self.goal_pub.publish(msg)
                return
            if not offboard:
                self.set_phase('CANCELLED', 'OFFBOARD/armed state ended or became stale; no automatic resume')
                return
            if self.phase == 'FLYING':
                arrived = now >= self.trajectory_end and self.at_rest(self.goal, now)
                if self.stable_for(arrived, self.goal_wait, now):
                    self.set_phase('LAND_REQUESTED')
                    threading.Thread(target=self.request_land, daemon=True).start()

    def request_land(self):
        try:
            self.mode_client.wait_for_service(timeout=2.)
            with self.lock:
                now = rospy.Time.now()
                if not (self.state and self.state.armed and self.state.mode == 'OFFBOARD'
                        and self.fresh(now, self.state_received, 3.) and self.at_rest(self.goal, now)):
                    self.set_phase('CANCELLED', 'Vehicle state changed before landing request')
                    return
            result = self.mode_client(base_mode=0, custom_mode='AUTO.LAND')
            with self.lock:
                if not result.mode_sent and self.phase == 'LAND_REQUESTED':
                    self.set_phase('FAILED', 'AUTO.LAND request rejected; no automatic retry')
        except (rospy.ROSException, rospy.ServiceException) as error:
            with self.lock:
                if self.phase == 'LAND_REQUESTED':
                    self.set_phase('FAILED', str(error))


if __name__ == '__main__':
    rospy.init_node('rrt_point_to_point_mission')
    mission = Mission()
    rospy.spin()
