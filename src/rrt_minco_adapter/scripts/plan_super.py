#!/usr/bin/env python3
"""Fixed-height SUPER bridge. Native ROS workspace stays in a child process."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rrt_core'))
from local_rrt_core import Scene
from minco_trajectory import certify_curve, bernstein_norm_bound


class Polynomial:
    def __init__(self, result):
        self.durations = np.asarray(result['duration'], dtype=float)
        self.coefficients = np.stack([np.asarray(result['coef_' + axis]).reshape(-1, 8)
                                      for axis in 'xyz'], axis=-1)
        if (not len(self.durations) or self.coefficients.shape[0] != len(self.durations)
                or not np.isfinite(self.coefficients).all()
                or not np.isfinite(self.durations).all() or np.any(self.durations <= 0)):
            raise ValueError('Invalid SUPER polynomial')
        self.ends = np.cumsum(self.durations)
        self.total_time = float(self.ends[-1])

    def derivative(self, t, order):
        i = min(int(np.searchsorted(self.ends, t, side='right')), len(self.durations)-1)
        local = np.clip(t-(self.ends[i-1] if i else 0), 0, self.durations[i])
        return np.array([np.polyval(np.polyder(self.coefficients[i, :, j], order), local)
                         for j in range(2)])

    def sample(self, t):
        return self.derivative(t, 0), self.derivative(t, 1)

    def controls(self):
        for coeff, duration in zip(self.coefficients, self.durations):
            powers = coeff[::-1] * duration**np.arange(8)[:, None]
            yield np.array([sum(math.comb(i, k)/math.comb(7, k)*powers[k]
                                for k in range(i+1)) for i in range(8)])


def validate_and_scale(result, data, start, goal, height, speed, acceleration, margin):
    traj = Polynomial(result)
    scene = Scene(data, clearance=margin)
    if (np.linalg.norm(traj.sample(0)[0]-start) > .01
            or np.linalg.norm(traj.sample(traj.total_time)[0]-goal) > .01):
        raise ValueError('SUPER endpoints mismatch')
    stretch = 1.
    for index, (b, duration) in enumerate(zip(traj.controls(), traj.durations)):
        if np.max(np.abs(b[:, 2]-height)) > 1e-5:
            raise ValueError('SUPER segment %d leaves fixed height' % index)
        if not certify_curve(b[:, :2], scene):
            reason = ('bounds' if not scene.within_bounds(b[:, :2].min(axis=0)) or
                      not scene.within_bounds(b[:, :2].max(axis=0)) else 'obstacle clearance')
            raise ValueError('SUPER segment %d fails continuous %s certification' % (index, reason))
        stretch = max(stretch, bernstein_norm_bound(7*np.diff(b[:, :2], axis=0)/duration)/speed,
                      math.sqrt(bernstein_norm_bound(42*np.diff(b[:, :2], n=2, axis=0)/duration**2)/acceleration))
    for i in range(len(traj.durations)-1):
        for order in range(4):
            for axis in range(3):
                left = np.polyval(np.polyder(traj.coefficients[i, :, axis], order), traj.durations[i])
                right = np.polyval(np.polyder(traj.coefficients[i+1, :, axis], order), 0.)
                if abs(left-right) > 1e-3:
                    raise ValueError('Discontinuous SUPER polynomial')
    result['native_duration'] = list(result['duration'])
    result['time_stretch'] = stretch
    result['duration'] = (traj.durations*stretch).tolist()
    for j, axis in enumerate('xyz'):
        result['coef_'+axis] = (traj.coefficients[:, :, j]/stretch**np.arange(7, -1, -1)).ravel().tolist()
    return Polynomial(result)


def solve(data, start=None, goal=None, speed=.2, acceleration=.8, clearance=.2, reserve=.1,
          height=.5):
    start = data['start'] if start is None else start
    goal = data['goal'] if goal is None else goal
    margin = clearance+reserve
    if not all(math.isfinite(v) and v > 0 for v in (speed, acceleration, margin)):
        raise ValueError('Invalid SUPER limits')
    scene = Scene(data, clearance=margin)
    if not scene.free(start, start) or not scene.free(goal, goal):
        raise ValueError('Start/goal in obstacle clearance or outside scene')
    bundle = Path(rospy.get_param('~super_bundle', str(Path.home()/'rrt_super_bundle'))).expanduser()
    binary = bundle/'adapter_ws/devel/lib/super_gazebo_adapter/super_static_planner_node'
    if not binary.is_file():
        raise RuntimeError('Build SUPER first: bash '+str(bundle/'scripts/build_super.sh'))
    began = time.monotonic()
    ident = 'super_' + uuid.uuid4().hex
    ns = '/rrt_comparison/' + ident
    directory = Path.home()/'.ros/super_matchws'/ident
    directory.mkdir(parents=True)
    # Use bundled adapter profile, then apply the same scene/limits as RRT.
    cfg = yaml.safe_load((bundle/'runtime/super_flight.yaml').read_text())
    cfg['super_planner'].update(robot_r=.05, planning_horizon=100., receding_dis=100.)
    cfg['traj_opt']['boundary'].update(max_vel=speed, max_acc=acceleration)
    cfg['traj_opt']['switch']['print_optimizer_log'] = False
    xmin, xmax, ymin, ymax = data['bounds']
    resolution = .1
    sizes = [max(20., xmax-xmin+4), max(20., ymax-ymin+4), max(4., abs(height)*2+4)]
    cfg['rog_map'].update(resolution=resolution, inflation_resolution=resolution,
        inflation_step=1, map_size=sizes,
        fix_map_origin=[(xmin+xmax)/2, (ymin+ymax)/2, height],
        virtual_ground_height=height-2-2*margin, virtual_ceil_height=height+2+2*margin)
    cfg['astar']['map_voxel_num'] = [int(math.ceil(s/resolution)) for s in sizes]
    config = directory/'config.yaml'
    config.write_text(yaml.safe_dump(cfg))
    (directory/'scene.json').write_text(json.dumps(data))
    points = []
    # Rectangles are [center_x,center_y,width,height]. Cover their area with occupied voxels.
    for x, y, width, depth in data['boxes']:
        # Encode Scene's axis-aligned clearance, not a different round-corner dilation.
        width += 2*margin
        depth += 2*margin
        for px in np.linspace(x-width/2, x+width/2, max(2, int(math.ceil(width/(resolution/2)))+1)):
            for py in np.linspace(y-depth/2, y+depth/2, max(2, int(math.ceil(depth/(resolution/2)))+1)):
                for pz in (height-resolution, height, height+resolution):
                    points.append((px, py, pz))
    if not points:
        # Native map requires a nonempty cloud; this voxel is outside allowed XY.
        points = [(xmin-1., ymin-1., height)]
    odom = Odometry(); odom.header.frame_id = 'world'
    odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, start)
    odom.pose.pose.position.z = height; odom.pose.pose.orientation.w = 1.
    fields = [PointField(name, i*4, PointField.FLOAT32, 1) for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    cloud = point_cloud2.create_cloud(Header(frame_id='world'), fields, [(*p, 1.) for p in points])
    target = PoseStamped(); target.header.frame_id = 'world'
    target.pose.position.x, target.pose.position.y = map(float, goal)
    target.pose.position.z = height; target.pose.orientation.w = 1.
    received = []; event = threading.Event()
    def on_result(msg):
        received.append(json.loads(msg.data)); event.set()
    sub = rospy.Subscriber(ns+'/polynomial', String, on_result, queue_size=1)
    pubs = [rospy.Publisher(ns+'/'+topic, typ, queue_size=1)
            for topic, typ in [('odom', Odometry), ('cloud', PointCloud2), ('goal', PoseStamped)]]
    env = dict(os.environ)
    for key in ('CMAKE_PREFIX_PATH', 'ROS_PACKAGE_PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH'):
        env.pop(key, None)
    args = ['bash', '-c', 'source "$1/adapter_ws/devel/setup.bash"; shift; exec "$@"',
            'super-worker', str(bundle), str(binary), '__name:='+ident,
            '_config_path:='+str(config)]
    args += ['/comparison/'+a+':='+ns+'/'+b for a, b in
             [('odom','odom'), ('cloud','cloud'), ('plan_goal','goal'), ('super_polynomial','polynomial')]]
    proc = None
    try:
        with (directory/'native.log').open('w') as log:
            proc = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+60
            while not all(p.get_num_connections() for p in pubs):
                if proc.poll() is not None or time.monotonic()>deadline or rospy.is_shutdown():
                    raise RuntimeError('SUPER worker startup failed; see '+str(directory/'native.log'))
                time.sleep(.05)
            # Native callback requires odometry before cloud. Publish snapshots in order.
            for _ in range(5):
                odom.header.stamp = rospy.Time.now(); pubs[0].publish(odom); time.sleep(.1)
                cloud.header.stamp = rospy.Time.now(); pubs[1].publish(cloud); time.sleep(.1)
            target.header.stamp = rospy.Time.now()
            requested = time.monotonic(); pubs[2].publish(target)
            while not event.wait(.1):
                if proc.poll() is not None or time.monotonic()-requested>60 or rospy.is_shutdown():
                    raise RuntimeError('SUPER planning timeout/exit; see '+str(directory/'native.log'))
            result = received[0]
            if result.get('order') != 7:
                raise RuntimeError('SUPER native planning failed; see '+str(directory/'native.log'))
            native_wall = time.monotonic()-requested
            (directory/'native_trajectory.json').write_text(json.dumps(result, indent=2))
            trajectory = validate_and_scale(result, data, start, goal, height, speed, acceleration, margin)
            result.update(planner='super', start=list(start), goal=list(goal),
                planning_clearance_m=margin, evaluation_clearance_m=clearance, tracking_margin_m=reserve,
                voxel_resolution_m=resolution, native_robot_r_m=.05,
                native_wall_s=native_wall, total_wall_s=time.monotonic()-began, native_log=str(directory/'native.log'))
            return trajectory, result
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try: proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL); proc.wait()
        sub.unregister()
        for publisher in pubs: publisher.unregister()
        if 'result' in locals() and 'total_wall_s' in result:
            result['total_wall_s'] = time.monotonic()-began
