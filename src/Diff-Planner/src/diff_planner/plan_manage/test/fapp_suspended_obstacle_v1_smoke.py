#!/usr/bin/env python3
"""Synthetic V1 integration checks on an isolated localhost ROS master.

Reuses the existing smoke fixture's ROS setup helpers and PolyTraj decoder.
The planner follows perfect synthetic odometry: these checks cover planner
interfaces and geometry, not sensor coverage, controller error, or real flight.
Published trajectories are checked against the observations' one-second CV
prediction; execution is checked against the actual fixture motion separately.
"""
import argparse
import json
import math
import os
import re
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import traceback

from fapp_dynamic_avoidance_smoke import (planner_parameters, poly_state,
                                         stop_process, surface_points)


SCENARIOS = ('default_disabled', 'underpass', 'left', 'right', 'multi',
             'swing_box', 'blocked', 'dropout', 'invalid_size', 'both_sides_cost')
RADIUS, HALF_HEIGHT, MARGIN = .262, .125, .10
BODY_HALF = (.185, .185, .125)
CORRIDOR = ((-.5, 7.), (-.75, .75), (0., 1.5))
START, GOAL = (0., 0., .85), (5., 0., .85)
GATE_BOXES = (
    ((1.65, 2.35), (-.75, .75), (0., .40)),
    ((1.65, 2.35), (-.75, .75), (1.40, 1.50)),
    ((1.65, 2.35), (-.75, -.55), (.40, 1.40)),
    ((1.65, 2.35), (.55, .75), (.40, 1.40)),
)


def box_bounds(center, size):
    return tuple((center[i] - size[i] / 2, center[i] + size[i] / 2)
                 for i in range(3))


def signed_product_distance(gaps):
    return math.sqrt(sum(max(gap, 0.) ** 2 for gap in gaps)) + min(0., max(gaps))


def cylinder_box_gap(position, box):
    """Exact separation of an upright cylinder and an axis-aligned box."""
    q = [abs(position[i] - (box[i][0] + box[i][1]) / 2) -
         (box[i][1] - box[i][0]) / 2 for i in range(3)]
    rectangle_distance = signed_product_distance(q[:2])
    return signed_product_distance((rectangle_distance - RADIUS,
                                    q[2] - HALF_HEIGHT))


def physical_box_gap(position, box):
    return signed_product_distance([
        abs(position[i] - (box[i][0] + box[i][1]) / 2) -
        (box[i][1] - box[i][0]) / 2 - BODY_HALF[i] for i in range(3)])


def cylinder_sphere_gap(position, center):
    horizontal = math.hypot(position[0] - center[0], position[1] - center[1]) - RADIUS
    vertical = abs(position[2] - center[2]) - HALF_HEIGHT
    return signed_product_distance((horizontal, vertical)) - .10


def static_gap(position, boxes, cylinder=True, corridor=CORRIDOR):
    half = (RADIUS, RADIUS, HALF_HEIGHT) if cylinder else BODY_HALF
    gaps = [position[i] - half[i] - corridor[i][0] for i in range(3)]
    gaps += [corridor[i][1] - position[i] - half[i] for i in range(3)]
    measure = cylinder_box_gap if cylinder else physical_box_gap
    gaps += [measure(position, box) for box in boxes]
    return min(gaps)


def fixtures(scenario, stamp, epoch, released=False):
    """(Center, velocity, actual size, transmitted size, shape) tuples."""
    if scenario in ('blocked', 'dropout'):
        center = (6., 0., .90) if released else (2., 0., .90)
        return [(center, (0., 0., 0.), (.25, .70, .65), (.25, .70, .65), 'box')]
    if scenario == 'invalid_size':
        return [((2., 0., .90), (0., 0., 0.), (.20, .20, .20),
                 (-.10, .20, .20), 'sphere')]
    if scenario == 'swing_box':
        phase = .65 * (stamp - epoch)
        return [((2., .1 + .12 * math.sin(phase), .95 + .03 * math.sin(phase) ** 2),
                 (0., .078 * math.cos(phase), .039 * math.sin(phase) * math.cos(phase)),
                 (.35, .30, .40), (.35, .30, .40), 'box')]
    if scenario == 'both_sides_cost':
        return [((2., .12, .90), (0., 0., 0.), (.35, .20, .90),
                 (.35, .20, .90), 'box')]
    if scenario in ('left', 'right'):
        y = -.30 if scenario == 'left' else .30
        return [((2., y, .90), (0., 0., 0.), (.35, .30, .90), (.35, .30, .90), 'box')]
    first = ((2., .30, .95), (0., 0., 0.), (.35, .30, .65), (.35, .30, .65), 'box')
    if scenario == 'multi':
        return [first, ((3.7, -.30, .90), (0., 0., 0.),
                        (.35, .30, .90), (.35, .30, .90), 'box')]
    return [first]


def scenario_corridor(scenario):
    # Wider fixture isolates cost comparison when both routes are feasible;
    # the primary integration scenarios retain the requested 1.5 m corridor.
    return ((-.5, 7.), (-1.1, 1.1), (0., 1.5)) if scenario == 'both_sides_cost' else CORRIDOR


def scenario_boxes(scenario):
    if scenario == 'both_sides_cost':
        return (((1.50, 2.50), (-1.1, 1.1), (0., .40)),)
    if scenario in ('blocked', 'dropout'):
        return GATE_BOXES
    if scenario in ('left', 'right'):
        return (((1.50, 2.50), (-.75, .75), (0., .40)),)
    if scenario == 'multi':
        return (((3.20, 4.20), (-.75, .75), (0., .40)),)
    return ()


def object_gap(position, center, size, shape='box'):
    if shape == 'sphere' or not (all(math.isfinite(v) and v >= 0 for v in size) and any(v > 0 for v in size)):
        return cylinder_sphere_gap(position, center)
    return cylinder_box_gap(position, box_bounds(center, size))


def physical_object_gap(position, center, size, shape):
    if shape == 'sphere':
        return math.sqrt(sum(max(abs(position[i] - center[i]) - BODY_HALF[i], 0.) ** 2
                             for i in range(3))) - .10
    return physical_box_gap(position, box_bounds(center, size))


def finite_min(value):
    return value if math.isfinite(value) else None


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

    enabled = args.scenario != 'default_disabled'
    boxes = scenario_boxes(args.scenario)
    corridor = scenario_corridor(args.scenario)
    result = dict(pass_=False, scenario=args.scenario, master_uri=os.environ['ROS_MASTER_URI'],
                  scope=__doc__, body_size=[.37, .37, .25], cylinder_radius=RADIUS,
                  cylinder_half_height=HALF_HEIGHT, required_margin=MARGIN,
                  sample_dt=.01, corridor_bounds=corridor, static_boxes=boxes,
                  trajectories=[], execution=[], observations=[], statuses=[], failures=[])
    lock, publishing = threading.Lock(), threading.Event()
    state = dict(position=START, velocity=(0., 0., 0.), trajectory=None,
                 epoch=None, goal_sent=None, observations=[], observation_stamp=None,
                 empty_since=None, released=False)
    master = node = publisher_thread = None
    master_log, node_log = (log_dir / 'master.log').open('w'), (log_dir / 'planner.log').open('w')
    try:
        master = subprocess.Popen(['roscore', '-p', str(port)], stdout=master_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 10
        while subprocess.run(['rosparam', 'list'], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL).returncode:
            assert master.poll() is None and time.monotonic() < deadline, 'Isolated ROS master failed'
            time.sleep(.05)
        rospy.init_node('fapp_suspended_obstacle_v1_smoke', anonymous=True, disable_signals=True)
        params = planner_parameters(args.workspace, enabled)
        params['optimization/max_planning_wall_time'] = .8
        if not enabled:
            # Exercise the actual false defaults, not an explicitly forced value.
            params.pop('fsm/enable_fapp_dynamic_avoidance')
            params.pop('optimization/enable_fapp_dynamic_avoidance')
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
        points = set()
        y_low, y_high = corridor[1]
        y_count = round((y_high - y_low) / .05)
        for index in range(151):
            x = -.5 + .05 * index
            points.update((x, y, .05 * z) for y in (y_low, y_high) for z in range(31))
            points.update((x, y_low + .05 * y, z) for y in range(y_count + 1) for z in (0., 1.5))
        for box in boxes:
            points.update(surface_points(box))
        cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='world'), sorted(points))

        def status_callback(message):
            with lock:
                result['statuses'].append(dict(time=rospy.Time.now().to_sec(), text=message.data))

        def trajectory_callback(message):
            try:
                now = rospy.Time.now().to_sec()
                assert message.order == 5 and message.duration, 'Invalid PolyTraj'
                assert all(math.isfinite(t) and t > 0 for t in message.duration), 'Invalid durations'
                duration = sum(message.duration)
                with lock:
                    observations = list(state['observations'])
                    observation_stamp = state['observation_stamp']
                    goal_sent = state['goal_sent']
                valid = enabled and observation_stamp is not None and now - observation_stamp <= .80
                static_min, physical_min, dynamic_min = math.inf, math.inf, math.inf
                crossings = {str(x): [] for x in (2., 3.7)}
                for index in range(math.ceil(duration / .01) + 1):
                    elapsed = min(duration, index * .01)
                    position = poly_state(message, elapsed)
                    assert all(map(math.isfinite, position)), 'Nonfinite trajectory'
                    static_min = min(static_min, static_gap(position, boxes, corridor=corridor))
                    physical_min = min(physical_min, static_gap(position, boxes, cylinder=False, corridor=corridor))
                    for center, speed, _, transmitted_size, shape in observations:
                        dt = max(0., min(1., message.start_time.to_sec() + elapsed - observation_stamp))
                        predicted = tuple(center[i] + dt * speed[i] for i in range(3))
                        dynamic_min = min(dynamic_min, object_gap(position, predicted, transmitted_size, shape))
                    for x in (2., 3.7):
                        if abs(position[0] - x) < .12:
                            crossings[str(x)].append(position)
                record = dict(received_time=now, start_time=message.start_time.to_sec(),
                              duration=duration, traj_id=message.traj_id,
                              goal_to_publish_seconds=now - goal_sent if goal_sent else None,
                              minimum_static_cylinder_clearance=static_min,
                              minimum_static_physical_box_clearance=physical_min,
                              minimum_cv_dynamic_clearance=finite_min(dynamic_min),
                              dynamic_constraint_expected=valid,
                              final_position=poly_state(message, duration),
                              crossings={x: dict(z_min=min((p[2] for p in points), default=None),
                                                 z_max=max((p[2] for p in points), default=None),
                                                 y_min=min((p[1] for p in points), default=None),
                                                 y_max=max((p[1] for p in points), default=None))
                                         for x, points in crossings.items()},
                              coefficients=dict(x=list(message.coef_x), y=list(message.coef_y),
                                                z=list(message.coef_z), duration=list(message.duration)))
                with lock:
                    result['trajectories'].append(record)
                    state['trajectory'] = message
                    if physical_min < -1e-5:
                        result['failures'].append('Published trajectory intersects physical static geometry')
                    if enabled and static_min < MARGIN - 1e-5:
                        result['failures'].append('Published trajectory violates complete-body static margin')
                    if valid and dynamic_min < MARGIN - 1e-5:
                        result['failures'].append('Published trajectory violates CV complete-body dynamic margin')
            except Exception as error:
                with lock:
                    result['failures'].append('Trajectory validation failed: ' + str(error))

        trajectory_sub = rospy.Subscriber('/fapp_fixture/trajectory', PolyTraj, trajectory_callback, queue_size=30)
        status_sub = rospy.Subscriber('/fapp_fixture/status', String, status_callback, queue_size=30)
        deadline = time.monotonic() + 10
        while not (odom_pub.get_num_connections() and cloud_pub.get_num_connections() and
                   goal_pub.get_num_connections() and (not enabled or objects_pub.get_num_connections())):
            assert node.poll() is None and time.monotonic() < deadline, 'Planner subscriptions missing'
            time.sleep(.02)
        if not enabled:
            assert objects_pub.get_num_connections() == 0, 'Default mode unexpectedly subscribes to dynamic states'
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
                    released = state['released']
                    epoch = state['epoch']
                actual = fixtures(args.scenario, stamp, epoch, released)
                odom = Odometry(header=Header(stamp=now, frame_id='world'))
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
                odom.pose.pose.orientation.w = 1.
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = velocity
                odom_pub.publish(odom)
                if count % 5 == 0:
                    cloud.header.stamp = now
                    cloud_pub.publish(cloud)
                    objects = ObjectsStates(header=Header(stamp=now, frame_id='world'))
                    if not empty:
                        for center, speed, _, size, _ in actual:
                            target = State(header=objects.header)
                            target.position.x, target.position.y, target.position.z = center
                            target.velocity.x, target.velocity.y, target.velocity.z = speed
                            target.size.x, target.size.y, target.size.z = size
                            objects.states.append(target)
                        with lock:
                            state['observations'], state['observation_stamp'] = actual, stamp
                            result['observations'].append(dict(stamp=stamp, states=actual))
                    objects_pub.publish(objects)
                if count % 5 == 0:
                    gap = min(object_gap(position, center, size, shape)
                              for center, _, size, _, shape in actual)
                    physical_gap = min(physical_object_gap(position, center, size, shape)
                                       for center, _, size, _, shape in actual)
                    with lock:
                        valid = enabled and (not empty or stamp - state['observation_stamp'] <= .80)
                        result['execution'].append(dict(time=stamp, position=position,
                                                        minimum_actual_dynamic_clearance=gap,
                                                        minimum_actual_physical_box_clearance=physical_gap,
                                                        dynamic_constraint_expected=valid))
                        if valid and gap < -1e-5:
                            result['failures'].append('Executed trajectory intersects actual fixture obstacle')
                count += 1
                time.sleep(.02)

        publisher_thread = threading.Thread(target=publish_fixture, daemon=True)
        publisher_thread.start()
        time.sleep(1.)
        goal = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='world'))
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = GOAL
        goal.pose.orientation.w = 1.
        with lock:
            state['goal_sent'] = rospy.Time.now().to_sec()
            result['goal_sent_time'] = state['goal_sent']
        goal_pub.publish(goal)
        deadline = time.monotonic() + args.timeout
        if args.scenario in ('blocked', 'dropout'):
            while time.monotonic() < deadline:
                with lock:
                    attempted = any('PLANNING_FAILED' in entry['text'] or 'SWING_OBSTACLE_WAIT' in entry['text']
                                    for entry in result['statuses'])
                    waited = rospy.Time.now().to_sec() - state['goal_sent'] >= 2.
                if attempted and waited:
                    break
                assert node.poll() is None, 'Planner exited while waiting'
                time.sleep(.02)
            assert attempted, 'No observed planning failure/wait for completely blocked aperture'
            with lock:
                assert state['position'][0] < 1.2, 'Vehicle entered the blocked aperture'
            if args.scenario == 'dropout':
                with lock:
                    state['empty_since'] = rospy.Time.now().to_sec()
                    result['empty_started'], result['last_nonempty_stamp'] = state['empty_since'], state['observation_stamp']
                time.sleep(.60)
                with lock:
                    assert state['position'][0] < 1.2, 'Empty array released cached obstacle too early'
                    assert not any(record['final_position'][0] > 2.5 and
                                   record['received_time'] <= result['last_nonempty_stamp'] + .80
                                   for record in result['trajectories']), 'Through trajectory published before 0.8-second expiry'
            else:
                time.sleep(.6)
                with lock:
                    assert state['position'][0] < 1.2, 'Vehicle left wait while obstacle remained'
                    state['released'] = True
                    result['obstacle_released_time'] = rospy.Time.now().to_sec()
        reached = False
        while time.monotonic() < deadline:
            with lock:
                reached = math.dist(state['position'], GOAL) < .12
            if reached:
                break
            assert node.poll() is None, 'Planner exited before goal'
            time.sleep(.02)
        assert reached, 'Planner did not reach goal'
        assert result['trajectories'], 'No actual PolyTraj published'
        crossings = [record['crossings']['2.0'] for record in result['trajectories']
                     if record['crossings']['2.0']['z_min'] is not None]
        assert crossings, 'No published trajectory crossed the obstacle region'
        if args.scenario == 'underpass':
            assert any(route['z_max'] <= .40 + 1e-5 and max(abs(route['y_min']), abs(route['y_max'])) < .15
                       for route in crossings), 'Downward route was not preferred although lower and side passages exist'
        if args.scenario == 'left':
            assert any(route['y_min'] > .18 for route in crossings), 'Expected the only feasible left (+Y) passage'
        if args.scenario == 'right':
            assert any(route['y_max'] < -.18 for route in crossings), 'Expected the only feasible right (-Y) passage'
        node_log.flush()
        candidate_cycles, cycle = [], {}
        for line in (log_dir / 'planner.log').read_text().splitlines():
            route = re.search(r'\] route=(underpass|left|right) success=([01]) cost=(\S+)', line)
            selected = re.search(r'\] selected=(underpass|left|right) cost=(\S+)', line)
            if route:
                name, success, cost = route.groups()
                if name == 'underpass':
                    cycle = {}
                cycle[name] = dict(success=success == '1', cost=finite_min(float(cost)))
            elif selected:
                name, cost = selected.groups()
                candidate_cycles.append(dict(routes=dict(cycle), selected=name, selected_cost=float(cost)))
        result['candidate_cycles'] = candidate_cycles
        if args.scenario == 'underpass':
            assert any(cycle['selected'] == 'underpass' and
                       not {'left', 'right'}.intersection(cycle['routes'])
                       for cycle in candidate_cycles), 'Valid underpass was not accepted before evaluating sides'
        if args.scenario == 'both_sides_cost':
            comparisons = [cycle for cycle in candidate_cycles
                           if all(cycle['routes'].get(side, {}).get('success') and
                                  cycle['routes'][side]['cost'] is not None for side in ('left', 'right'))]
            assert comparisons, 'Both side candidates were not independently optimized and validated'
            for cycle in comparisons:
                minimum = min(cycle['routes'][side]['cost'] for side in ('left', 'right'))
                assert abs(cycle['selected_cost'] - minimum) <= 2e-6, \
                    'Selected side did not have the lowest existing trajectory cost'
                assert cycle['selected'] in ('left', 'right'), 'Underpass chosen despite blocked lower passage'
        if args.scenario == 'multi':
            assert any(not cycle['routes'].get('underpass', {}).get('success') and
                       cycle['routes'].get('underpass', {}).get('cost') is not None and
                       cycle['selected'] in ('left', 'right') for cycle in candidate_cycles), \
                'No side fallback after downward search reached optimization but failed acceptance'
            second = [record['crossings']['3.7'] for record in result['trajectories']]
            assert any(route['y_min'] is not None and route['y_min'] > .18 for route in second), 'Second target was not avoided'
        if args.scenario == 'default_disabled':
            assert any(record['minimum_cv_dynamic_clearance'] is not None and
                       record['minimum_cv_dynamic_clearance'] < 0 for record in result['trajectories']), \
                'Default-disabled fixture did not distinguish detection-only from avoidance'
        if args.scenario == 'dropout':
            through = [record['received_time'] for record in result['trajectories'] if record['final_position'][0] > 2.5]
            assert through and min(through) > result['last_nonempty_stamp'] + .80, 'Released before retention boundary'
            result['release_delay'] = min(through) - result['last_nonempty_stamp']
        assert not result['failures'], '; '.join(sorted(set(result['failures'])))
        result['pass_'] = True
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
    parser.add_argument('--scenario', choices=('all',) + SCENARIOS, default='all')
    parser.add_argument('--workspace', type=Path, default=Path('/home/oem/match_ws'))
    parser.add_argument('--node', type=Path)
    parser.add_argument('--port', type=int, default=0)
    parser.add_argument('--timeout', type=float, default=35.)
    parser.add_argument('--log-dir', type=Path)
    args = parser.parse_args()
    args.node = args.node or args.workspace / 'devel/lib/diff_planner/diff_planner_node'
    log_dir = args.log_dir or Path(tempfile.mkdtemp(prefix='fapp_suspended_v1_'))
    log_dir.mkdir(parents=True, exist_ok=True)
    if args.scenario == 'all':
        results = {}
        for scenario in SCENARIOS:
            child_dir = log_dir / scenario
            child_dir.mkdir(exist_ok=True)
            command = [sys.executable, str(Path(__file__).resolve()), '--scenario', scenario,
                       '--workspace', str(args.workspace), '--node', str(args.node),
                       '--timeout', str(args.timeout), '--port', str(args.port), '--log-dir', str(child_dir)]
            results[scenario] = subprocess.run(command).returncode == 0
        (log_dir / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
        print('SUMMARY {} log={}'.format(json.dumps(results), log_dir), flush=True)
        return 0 if all(results.values()) else 1
    return 0 if run_scenario(args, log_dir) else 1


if __name__ == '__main__':
    sys.exit(main())
