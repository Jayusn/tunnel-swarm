#!/usr/bin/env python3
"""Exercise the built Diff planner on an isolated localhost ROS master.

This fixture publishes synthetic sensor messages and follows the planner's
actual PolyTraj output.  It never launches a controller, simulator, or MAVROS.
Collision checks use the specified physical box and sphere, independently of
the optimizer's clearance function.  Results are synthetic integration checks,
not a claim about perception, tracking error, or real flight performance.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import traceback
import xml.etree.ElementTree as ET


SCENARIOS = ('baseline', 'disabled', 'swing', 'blocked', 'dropout',
             'planner_priority', 'priority_static')
HALF_BODY = (.185, .185, .125)
BALL_RADIUS = .10
START = (0., 0., .60)
GOAL = (5., 0., .60)
CORRIDOR = ((-.5, 7.), (-.75, .75), (0., 1.5))
# A static aperture admits the vehicle without the ball, but its lower,
# upper, and side gaps around the ball are all narrower than the vehicle.
GATE_BOXES = (
    ((1.70, 2.30), (-.75, .75), (0., .58)),
    ((1.70, 2.30), (-.75, .75), (1.15, 1.50)),
    ((1.70, 2.30), (-.75, -.35), (.58, 1.15)),
    ((1.70, 2.30), (.35, .75), (.58, 1.15)),
)


def poly_state(message, elapsed, derivative=0):
    """Decode the descending power coefficients published in PolyTraj."""
    piece = 0
    local = max(0., min(elapsed, sum(message.duration)))
    while piece < len(message.duration) - 1 and local > message.duration[piece]:
        local -= message.duration[piece]
        piece += 1
    values = []
    width = message.order + 1
    for coefficients in (message.coef_x, message.coef_y, message.coef_z):
        value = 0.
        for index in range(width - derivative):
            exponent = message.order - index
            multiplier = math.prod(range(exponent - derivative + 1, exponent + 1))
            value = value * local + coefficients[piece * width + index] * multiplier
        values.append(value)
    return tuple(values)


def sphere_box_gap(position, center):
    squared = sum(max(abs(position[i] - center[i]) - HALF_BODY[i], 0.) ** 2
                  for i in range(3))
    return math.sqrt(squared) - BALL_RADIUS


def static_clearance(position, gate, wall=False):
    clearance = min(position[i] - HALF_BODY[i] - CORRIDOR[i][0]
                    for i in range(3))
    clearance = min(clearance, *(CORRIDOR[i][1] - position[i] - HALF_BODY[i]
                                 for i in range(3)))
    if gate:
        boxes = GATE_BOXES + ((((1.70, 2.30), (-.75, .75), (0., 1.50)),) if wall else ())
        for box in boxes:
            # At least one separating axis is necessary for disjoint boxes.
            separation = max(box[i][0] - position[i] - HALF_BODY[i]
                             if position[i] < box[i][0]
                             else position[i] - HALF_BODY[i] - box[i][1]
                             for i in range(3))
            clearance = min(clearance, separation)
    return clearance


def ball_state(stamp, epoch, swing):
    if not swing:
        return (2., 0., .90), (0., 0., 0.)
    phase = .90 * (stamp - epoch)
    s, c = math.sin(phase), math.cos(phase)
    return (2., .45 * s, .90 + .15 * s * s), (0., .405 * c, .27 * s * c)


def surface_points(box, step=.05):
    axes = [[lo + index * (hi - lo) / max(1, math.ceil((hi - lo) / step))
             for index in range(max(1, math.ceil((hi - lo) / step)) + 1)]
            for lo, hi in box]
    points = set()
    for dimension in range(3):
        others = [axis for axis in range(3) if axis != dimension]
        for boundary in box[dimension]:
            for a in axes[others[0]]:
                for b in axes[others[1]]:
                    point = [0., 0., 0.]
                    point[dimension], point[others[0]], point[others[1]] = boundary, a, b
                    points.add(tuple(point))
    return points


def planner_parameters(workspace, enabled):
    path = workspace / 'src/Diff-Planner/src/diff_planner/plan_manage/launch/include/advanced_param_sim.xml'
    arguments = dict(drone_id='0', flight_type='1', planning_horizon='6.0',
                     point_num='1', max_vel='0.50', max_acc='0.50', max_jer='20.0',
                     map_size_x_='16.0', map_size_y_='4.0', map_size_z_='3.0',
                     cx='321.046', cy='243.450', fx='387.229', fy='387.229',
                     use_multitopology_trajs='false')
    for index in range(5):
        for axis in 'xyz':
            arguments['point{}_{}'.format(index, axis)] = '0.0'
    params = {}
    for element in ET.parse(path).findall('.//node/param'):
        value = re.sub(r'\$\(arg ([^)]+)\)', lambda match: arguments[match[1]], element.attrib['value'])
        kind = element.get('type')
        if value.lower() in ('true', 'false'):
            value = value.lower() == 'true'
        elif kind == 'int':
            value = int(value)
        else:
            try:
                value = float(value)
            except ValueError:
                pass
        params[element.attrib['name']] = value
    params.update({
        'fsm/manual_goal_topic': '/fapp_fixture/goal',
        'fsm/require_pre_agent_trajectory': False,
        'fsm/enable_periodic_replan': False,
        'fsm/enable_random_global_init': False,
        'fsm/enable_random_local_init': False,
        'fsm/enable_stuck_detect': False,
        'fsm/mondify_final_goal': False,
        'fsm/enable_occupied_recovery': False,
        'fsm/enable_fapp_dynamic_avoidance': enabled,
        'fsm/fapp_obstacle_topic': '/UAV1/fapp/states',
        'fsm/enable_swing_obstacle_guard': False,
        'fsm/max_tracking_error': .30,
        'grid_map/resolution': .05,
        'grid_map/local_update_range_x': 8.,
        'grid_map/local_update_range_y': 1.5,
        'grid_map/local_update_range_z': 1.5,
        'grid_map/obstacles_inflation': .099,
        'grid_map/cloud_use_init_offset': False,
        'grid_map/cloud_enable_raycast': False,
        'grid_map/preserve_global_map': True,
        'grid_map/enable_virtual_wall': False,
        'grid_map/show_occ_time': False,
        'grid_map/frame_id': 'world',
        'grid_map/fading_time': 0.,
        'manager/enable_replan_quality_gate': False,
        'optimization/constraint_points_perPiece': 6,
        'optimization/weight_obstacle': 10000.,
        'optimization/weight_obstacle_soft': 10000.,
        'optimization/weight_feasibility': 10000.,
        'optimization/weight_swarm': 10000.,
        'optimization/weight_time': 5.,
        'optimization/obstacle_clearance': .15,
        'optimization/obstacle_clearance_soft': .30,
        'optimization/record_opt': False,
        'optimization/enable_fapp_dynamic_avoidance': enabled,
        'optimization/fapp_vehicle_radius': .262,
        'optimization/fapp_vehicle_half_height': .125,
        'optimization/fapp_ball_radius': .10,
        'optimization/fapp_margin': .10,
        'optimization/fapp_observation_retention': .80,
        'optimization/fapp_prediction_horizon': 1.0,
        'optimization/fapp_minimum_ball_bottom': .80,
        'optimization/fapp_ground_height': 0.,
        'optimization/fapp_underpass_height': .50,
        'optimization/weight_dynamic_obstacle': 10000.,
    })
    return params


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        os.killpg(process.pid, sig)
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass


def run_scenario(args, log_dir):
    with socket.socket() as guard:
        guard.bind(('127.0.0.1', args.port))
        port = guard.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:' + str(port),
                      ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
                      ROS_LOG_DIR=str(log_dir / 'roslogs'))
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from obj_state_msgs.msg import ObjectsStates, State
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header, String
    from traj_utils.msg import PolyTraj

    enabled = args.scenario != 'disabled'
    priority = args.scenario in ('planner_priority', 'priority_static')
    gate = args.scenario in ('blocked', 'dropout', 'gate_baseline') or priority
    static_wall = args.scenario == 'priority_static'
    dynamic = args.scenario not in ('baseline', 'gate_baseline')
    swing = args.scenario in ('swing', 'disabled')
    # Keep the gate fixture at its static aperture height, separating dynamic
    # blocking/expiry from the existing static planner's step-climbing behavior.
    start_position = (0., 0., .85) if gate else START
    goal_position = (5., 0., .85) if gate else GOAL
    result = dict(pass_=False, scenario=args.scenario, master_uri=os.environ['ROS_MASTER_URI'],
                  scope=__doc__, physical_body_size=[.37, .37, .25], ball_diameter=.20,
                  trajectory_sample_step=.01, geometric_float_tolerance=1e-5,
                  start_position=start_position, goal_position=goal_position,
                  corridor_bounds=CORRIDOR, gate_boxes=GATE_BOXES if gate else [],
                  trajectories=[], statuses=[], execution=[], failures=[])
    node = master = None
    publishing = threading.Event()
    publisher_thread = None
    lock = threading.Lock()
    state = dict(position=start_position, velocity=(0., 0., 0.), trajectory=None, empty_since=None,
                 last_nonempty=None, epoch=None)
    master_log = (log_dir / 'master.log').open('w')
    node_log = (log_dir / 'planner.log').open('w')
    try:
        master = subprocess.Popen(['roscore', '-p', str(port)], stdout=master_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 10
        while subprocess.run(['rosparam', 'list'], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL).returncode:
            assert master.poll() is None and time.monotonic() < deadline, 'Isolated ROS master failed'
            time.sleep(.05)
        rospy.init_node('fapp_dynamic_avoidance_smoke', anonymous=True, disable_signals=True)
        params = planner_parameters(args.workspace, enabled)
        params['optimization/fapp_planner_priority'] = priority
        for name, value in params.items():
            rospy.set_param('/fapp_fixture_planner/' + name, value)
        (log_dir / 'parameters.json').write_text(json.dumps(params, indent=2) + '\n')
        command = [str(args.node), '__name:=fapp_fixture_planner',
                   '~odom_world:=/fapp_fixture/odom', '~grid_map/odom:=/fapp_fixture/odom',
                   '~grid_map/cloud:=/fapp_fixture/cloud',
                   '~planning/trajectory:=/fapp_fixture/trajectory',
                   '~planning/status:=/fapp_fixture/status']
        result['node_command'] = command
        node = subprocess.Popen(command, stdout=node_log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        odom_pub = rospy.Publisher('/fapp_fixture/odom', Odometry, queue_size=10)
        cloud_pub = rospy.Publisher('/fapp_fixture/cloud', PointCloud2, queue_size=2)
        objects_pub = rospy.Publisher('/UAV1/fapp/states', ObjectsStates, queue_size=10)
        goal_pub = rospy.Publisher('/fapp_fixture/goal', PoseStamped, queue_size=1)
        # Open ends are intentional; x bounds are checked independently below.
        points = set()
        x_values = [-.5 + .05 * index for index in range(151)]
        y_values = [-.75 + .05 * index for index in range(31)]
        z_values = [.05 * index for index in range(31)]
        for x in x_values:
            points.update((x, y, z) for y in (-.75, .75) for z in z_values)
            points.update((x, y, z) for y in y_values for z in (0., 1.5))
        if gate:
            for box in GATE_BOXES:
                points.update(surface_points(box))
        if static_wall:
            points.update(surface_points(((1.70, 2.30), (-.75, .75), (0., 1.50))))
        cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='world'), sorted(points))

        def on_status(message):
            with lock:
                result['statuses'].append(dict(time=rospy.Time.now().to_sec(), text=message.data))

        def on_trajectory(message):
            now = rospy.Time.now().to_sec()
            assert message.order == 5 and message.duration, 'Invalid PolyTraj'
            duration = sum(message.duration)
            assert all(math.isfinite(t) and t > 0 for t in message.duration), 'Invalid segment duration'
            samples = math.ceil(duration / .01)
            static_min, dynamic_min = math.inf, math.inf
            crossing = []
            for index in range(samples + 1):
                elapsed = min(duration, index * duration / samples)
                position = poly_state(message, elapsed)
                assert all(map(math.isfinite, position)), 'Nonfinite PolyTraj coefficients'
                static_min = min(static_min, static_clearance(position, gate, static_wall))
                if dynamic:
                    center, _ = ball_state(message.start_time.to_sec() + elapsed, state['epoch'], swing)
                    dynamic_min = min(dynamic_min, sphere_box_gap(position, center))
                if abs(position[0] - 2.) < .25:
                    crossing.append(position)
            with lock:
                empty_since = state['empty_since']
                valid_ball = dynamic and enabled and (empty_since is None or now - state['last_nonempty'] <= .80)
                record = dict(received_time=now, start_time=message.start_time.to_sec(),
                              duration=duration, traj_id=message.traj_id,
                              minimum_static_clearance=static_min,
                              minimum_physical_ball_clearance=dynamic_min if dynamic else None,
                              ball_constraint_expected=valid_ball and not priority,
                              final_position=poly_state(message, duration),
                              crossing_z_min=min((p[2] for p in crossing), default=None),
                              crossing_z_max=max((p[2] for p in crossing), default=None),
                              crossing_y_max=max((abs(p[1]) for p in crossing), default=None),
                              coefficients=dict(x=list(message.coef_x), y=list(message.coef_y),
                                                z=list(message.coef_z), duration=list(message.duration)))
                result['trajectories'].append(record)
                if static_min < -1e-5:
                    result['failures'].append('Published trajectory intersects physical static geometry')
                if valid_ball and not priority and dynamic_min < -1e-5:
                    result['failures'].append('Published trajectory intersects physical ball')
                state['trajectory'] = message

        def checked_trajectory(message):
            try:
                on_trajectory(message)
            except Exception as error:
                with lock:
                    result['failures'].append('Invalid planner trajectory: ' + str(error))

        trajectory_sub = rospy.Subscriber('/fapp_fixture/trajectory', PolyTraj, checked_trajectory, queue_size=30)
        status_sub = rospy.Subscriber('/fapp_fixture/status', String, on_status, queue_size=30)
        deadline = time.monotonic() + 10
        # roscpp shares one transport connection for both odometry callbacks
        # subscribed by the same planner process.
        while not (odom_pub.get_num_connections() and cloud_pub.get_num_connections() and
                   goal_pub.get_num_connections() and (not enabled or objects_pub.get_num_connections())):
            assert node.poll() is None and time.monotonic() < deadline, 'Planner input subscriptions missing'
            time.sleep(.02)
        result['ros_nodes'] = subprocess.check_output(['rosnode', 'list'], text=True).splitlines()
        assert all('mavros' not in name.lower() and 'px4' not in name.lower() for name in result['ros_nodes'])
        state['epoch'] = rospy.Time.now().to_sec()
        publishing.set()

        def publish_fixture():
            count = 0
            while publishing.is_set() and not rospy.is_shutdown():
                now = rospy.Time.now()
                stamp = now.to_sec()
                with lock:
                    trajectory = state['trajectory']
                    if trajectory is not None and stamp >= trajectory.start_time.to_sec():
                        elapsed = stamp - trajectory.start_time.to_sec()
                        state['position'] = poly_state(trajectory, elapsed)
                        state['velocity'] = poly_state(trajectory, elapsed, 1)
                        if elapsed >= sum(trajectory.duration):
                            state['velocity'] = (0., 0., 0.)
                    position, velocity = state['position'], state['velocity']
                    empty = state['empty_since'] is not None
                odom = Odometry(header=Header(stamp=now, frame_id='world'))
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
                odom.pose.pose.orientation.w = 1.
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = velocity
                odom_pub.publish(odom)
                if count % 5 == 0:
                    cloud.header.stamp = now
                    cloud_pub.publish(cloud)
                    objects = ObjectsStates(header=Header(stamp=now, frame_id='world'))
                    if dynamic and not empty:
                        center, speed = ball_state(stamp, state['epoch'], swing)
                        ball = State(header=objects.header)
                        ball.position.x, ball.position.y, ball.position.z = center
                        ball.velocity.x, ball.velocity.y, ball.velocity.z = speed
                        ball.size.x = ball.size.y = ball.size.z = .20
                        objects.states.append(ball)
                        with lock:
                            state['last_nonempty'] = stamp
                    objects_pub.publish(objects)
                if count % 5 == 0:
                    with lock:
                        center = ball_state(stamp, state['epoch'], swing)[0] if dynamic else None
                        result['execution'].append(dict(time=stamp, position=position, ball_position=center))
                count += 1
                time.sleep(.02)

        publisher_thread = threading.Thread(target=publish_fixture, daemon=True)
        publisher_thread.start()
        time.sleep(1.)
        goal = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='world'))
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = goal_position
        goal.pose.orientation.w = 1.
        goal_pub.publish(goal)
        goal_sent = rospy.Time.now().to_sec()
        deadline = time.monotonic() + args.timeout
        if gate and dynamic and (not priority or static_wall):
            # Initial planning can keep the vehicle stationary without first
            # publishing a moving trajectory.  Do not require a new FSM state
            # when the existing retry path already implements that wait.
            attempted = False
            while time.monotonic() < deadline:
                with lock:
                    attempted = any('SWING_OBSTACLE_WAIT' in item['text'] or
                                    'PLANNING_FAILED' in item['text']
                                    for item in result['statuses'])
                if attempted and rospy.Time.now().to_sec() - goal_sent >= 2.:
                    break
                assert node.poll() is None, 'Planner exited'
                time.sleep(.02)
            assert attempted, 'No observed planning attempt for the blocked aperture'
            time.sleep(.3)
            with lock:
                assert state['position'][0] < 1.5, 'Vehicle proceeded through blocked aperture'
            if args.scenario == 'dropout':
                with lock:
                    state['empty_since'] = rospy.Time.now().to_sec()
                    last_nonempty = state['last_nonempty']
                    result['last_nonempty_stamp'] = last_nonempty
                    result['empty_started'] = state['empty_since']
                time.sleep(.60)
                with lock:
                    assert state['position'][0] < 1.5, 'Empty states released the obstacle before retention elapsed'
                    assert not any(item['received_time'] <= last_nonempty + .80 and
                                   item['final_position'][0] > 2.5 for item in result['trajectories']), \
                        'A through trajectory was published before observation expiry'
                while time.monotonic() < deadline:
                    with lock:
                        reached = math.dist(state['position'], goal_position) < .12
                    if reached:
                        break
                    assert node.poll() is None, 'Planner exited'
                    time.sleep(.02)
                assert reached, 'No static-feasible route after 0.8-second observation expiry'
                resumed = [item for item in result['trajectories'] if item['final_position'][0] > 2.5]
                assert resumed and min(item['received_time'] for item in resumed) > last_nonempty + .80, \
                    'Released before the 0.8-second retention boundary'
                result['release_delay'] = min(item['received_time'] for item in resumed) - last_nonempty
            else:
                time.sleep(1.2)
                with lock:
                    assert state['position'][0] < 1.5, 'Vehicle left wait while aperture remained blocked'
        else:
            while time.monotonic() < deadline:
                with lock:
                    reached = math.dist(state['position'], goal_position) < .12
                if reached:
                    break
                assert node.poll() is None, 'Planner exited'
                time.sleep(.02)
            assert reached, 'Planner did not reach the requested goal'
            assert result['trajectories'], 'No real PolyTraj was published'
            if args.scenario == 'swing':
                assert any(item['crossing_z_min'] is not None for item in result['trajectories']), \
                    'No trajectory crossed the ball region'
        if args.scenario == 'planner_priority':
            assert any(item['minimum_physical_ball_clearance'] < 0.
                       for item in result['trajectories']), 'Fixture did not exercise FAPP overlap'
            assert not any('SWING_OBSTACLE_WAIT' in item['text']
                           for item in result['statuses']), 'FAPP still interrupted execution'
        assert not result['failures'], '; '.join(sorted(set(result['failures'])))
        result['pass_'] = True
        result['goal_sent_time'] = goal_sent
        print('PASS {} trajectories={} log={}'.format(args.scenario, len(result['trajectories']), log_dir), flush=True)
    except Exception as error:
        result['failures'].append(str(error))
        result['traceback'] = traceback.format_exc()
        print('FAIL {}: {} log={}'.format(args.scenario, error, log_dir), flush=True)
    finally:
        publishing.clear()
        if publisher_thread is not None:
            publisher_thread.join(timeout=2)
        stop_process(node)
        stop_process(master)
        node_log.close()
        master_log.close()
        (log_dir / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    return result['pass_']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('all', 'gate_baseline') + SCENARIOS, default='all')
    parser.add_argument('--workspace', type=Path, default=Path('/home/oem/match_ws'))
    parser.add_argument('--node', type=Path)
    parser.add_argument('--port', type=int, default=0, help='0 selects a free localhost port')
    parser.add_argument('--timeout', type=float, default=30.)
    parser.add_argument('--log-dir', type=Path)
    args = parser.parse_args()
    args.node = args.node or args.workspace / 'devel/lib/diff_planner/diff_planner_node'
    log_dir = args.log_dir or Path(tempfile.mkdtemp(prefix='fapp_dynamic_avoidance_'))
    log_dir.mkdir(parents=True, exist_ok=True)
    if args.scenario == 'all':
        results = {}
        for scenario in SCENARIOS:
            child_dir = log_dir / scenario
            child_dir.mkdir(exist_ok=True)
            command = [sys.executable, str(Path(__file__).resolve()), '--scenario', scenario,
                       '--workspace', str(args.workspace), '--node', str(args.node),
                       '--timeout', str(args.timeout), '--port', str(args.port),
                       '--log-dir', str(child_dir)]
            results[scenario] = subprocess.run(command).returncode == 0
        (log_dir / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
        print('SUMMARY {} log={}'.format(json.dumps(results), log_dir), flush=True)
        return 0 if all(results.values()) else 1
    return 0 if run_scenario(args, log_dir) else 1


if __name__ == '__main__':
    sys.exit(main())
