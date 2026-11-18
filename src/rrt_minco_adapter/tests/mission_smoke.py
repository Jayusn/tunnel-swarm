#!/usr/bin/env python3
"""Real RRT/node chain with simulated command-following odometry and MAVROS services."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client
from ros_smoke import wait_for


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mpc-ch10", action="store_true", help="真实MPC接收RRT参考并验证CH10中断，模拟传感器")
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ['ROS_IP'] = '127.0.0.1'
    directory = Path(tempfile.mkdtemp(prefix='rrt_mission_test_'))
    os.environ['ROS_LOG_DIR'] = str(directory / 'ros_logs')
    processes, files = [], []
    try:
        def spawn(command, filename):
            output = open(directory / filename, 'w')
            files.append(output)
            processes.append(subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        spawn(['roscore', '-p', str(port)], 'master.log')

        def ready():
            try:
                return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')[0] == 1
            except OSError:
                return False
        wait_for(ready)
        import rospy
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State, ExtendedState, RCIn, AttitudeTarget, ESCStatus, ESCStatusItem
        from mavros_msgs.srv import SetMode, SetModeResponse
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand
        from sensor_msgs.msg import PointCloud2, Imu
        from sensor_msgs import point_cloud2
        from std_msgs.msg import String, Header, Bool
        from rosgraph_msgs.msg import Log
        from std_srvs.srv import Trigger
        rospy.init_node('rrt_mission_test', anonymous=True, disable_signals=True)
        state = State(connected=True, armed=False, mode='POSCTL')
        extended = ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
        odom = Odometry()
        odom.header.frame_id = 'UAV0/camera_init'
        odom.pose.pose.position.x = odom.pose.pose.position.y = .01
        odom.pose.pose.orientation.w = 1.
        rc = RCIn(channels=[1500]*7+[1000,1500,1000])
        rc_pub = rospy.Publisher('/UAV0/mavros/rc/in', RCIn, queue_size=1)
        state_pub = rospy.Publisher('/UAV0/mavros/state', State, queue_size=1)
        ext_pub = rospy.Publisher('/UAV0/mavros/extended_state', ExtendedState, queue_size=1)
        odom_pub = rospy.Publisher('/UAV0/fast_lio/Odom_high_freq', Odometry, queue_size=1)
        map_odom_pub = rospy.Publisher('/UAV0/fast_lio/Odometry', Odometry, queue_size=1)
        cloud_pub = rospy.Publisher('/UAV0/fast_lio/cloud_registered', PointCloud2, queue_size=1)
        goals, phases, mode_requests, trajectories = [], [], [], []
        commands, attitudes, rc_lands, mpc_logs = [], [], [], []
        imu = Imu()
        imu.orientation.w = 1.
        imu.linear_acceleration.z = 9.795
        esc = ESCStatus(esc_status=[ESCStatusItem(rpm=15788) for _ in range(4)])
        imu_pub = rospy.Publisher('/UAV0/livox/imu_mps2', Imu, queue_size=1)
        esc_pub = rospy.Publisher('/UAV0/mavros/esc_status', ESCStatus, queue_size=1)
        local_pub = rospy.Publisher('/UAV0/mavros/local_position/pose', PoseStamped, queue_size=1)

        def command(msg):
            commands.append(msg)
            if state.mode == 'OFFBOARD':
                odom.pose.pose.position = msg.position
                odom.twist.twist.linear = msg.velocity

        def request_mode(req):
            assert req.custom_mode == 'AUTO.LAND'
            mode_requests.append((time.monotonic(), req.custom_mode))
            state.mode = 'AUTO.LAND'
            return SetModeResponse(mode_sent=True)

        service = rospy.Service('/UAV0/mavros/set_mode', SetMode, request_mode)
        subscribers = [
            rospy.Subscriber('/UAV0/rrt/test_pos_cmd', PositionCommand, command),
            rospy.Subscriber('/UAV0/rrt/mission_goal', PoseStamped, lambda m: goals.append((time.monotonic(), m))),
            rospy.Subscriber('/UAV0/rrt/polynomial', String, lambda m: trajectories.append(json.loads(m.data))),
            rospy.Subscriber('/UAV0/rrt/mission_status', String, lambda m: phases.append(json.loads(m.data)['phase'])),
            rospy.Subscriber('/UAV0/mavros/setpoint_raw/attitude', AttitudeTarget, attitudes.append),
            rospy.Subscriber('/UAV0/mission/rc_land_active', Bool, lambda m: rc_lands.append(m.data)),
            rospy.Subscriber('/rosout_agg', Log, lambda m: mpc_logs.append(m.msg) if m.name == '/mpc_controller_node' else None)]

        last_state_time = [0.]
        def heartbeat(_):
            odom.header.stamp = rospy.Time.now()
            if time.monotonic() - last_state_time[0] >= 1.:
                state_pub.publish(state)
                ext_pub.publish(extended)
                last_state_time[0] = time.monotonic()
            rc.header.stamp = rospy.Time.now()
            rc_pub.publish(rc)
            odom_pub.publish(odom)
            map_odom_pub.publish(odom)
            if args.mpc_ch10:
                imu.header.stamp = esc.header.stamp = rospy.Time.now()
                imu_pub.publish(imu)
                esc_pub.publish(esc)
                local_pub.publish(PoseStamped(header=odom.header, pose=odom.pose.pose))
        timer = rospy.Timer(rospy.Duration(.02), heartbeat)
        if args.mpc_ch10:
            spawn(['roslaunch', 'rrt_minco_adapter', 'autotrans_controller.launch'], 'mpc.log')
            # MPC现有参考接口重映射到测试专用话题，无实际飞控。
            mpc_cmd_pub = rospy.Publisher('/UAV0/planning/pos_cmd', PositionCommand, queue_size=1)
            subscribers.append(rospy.Subscriber('/UAV0/rrt/test_pos_cmd', PositionCommand,
                                                lambda m: mpc_cmd_pub.publish(m)))
        spawn(['rosrun', 'rrt_minco_adapter', 'start_flight.sh',
               'command_topic:=/UAV0/rrt/test_pos_cmd'], 'mission.log')
        rospy.wait_for_service('/UAV0/rrt/freeze_map', timeout=15)
        wait_for(lambda: cloud_pub.get_num_connections() > 0 and 'WAIT_START' in phases)
        for _ in range(12):
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=rospy.Time.now(), frame_id='UAV0/camera_init'), [(2., 1., .5)]))
            time.sleep(.15)
        assert rospy.ServiceProxy('/UAV0/rrt/freeze_map', Trigger)().success
        time.sleep(2.2)
        assert not goals and not mode_requests, 'Must wait for manual arming/OFFBOARD'
        state.armed, state.mode = True, 'OFFBOARD'
        extended.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        odom.pose.pose.position.z = .5
        time.sleep(2.3)
        assert not goals, "CH8 low must not start RRT execution clock"
        rc.channels[7] = 2000
        ready_at = time.monotonic()
        wait_for(lambda: goals, timeout=8)
        assert len(goals) == 1 and goals[0][0] - ready_at < 2.
        goal = goals[0][1].pose.position
        assert (goal.x, goal.y, goal.z) == (3., 3., .5)
        wait_for(lambda: trajectories, timeout=15)
        result = trajectories[-1]
        assert result['order'] == 5, result
        assert result['start'] == [.01, .01]
        assert result['goal'] == [3., 3.]
        print('Trajectory ready; testing full %.2fs flight plus 2s goal hover' % sum(result['duration']), flush=True)
        if args.mpc_ch10:
            wait_for(lambda: any('收到 PositionCommand' in m for m in mpc_logs), timeout=15)
            time.sleep(1.)
            assert attitudes and 0. < attitudes[-1].thrust < 1.
            assert commands[-1].position.x > .01 or commands[-1].position.y > .01
            rc.channels[9] = 2000
            wait_for(lambda: True in rc_lands and 'CANCELLED' in phases)
            time.sleep(.3)
            count = len(commands)
            time.sleep(.7)
            assert len(commands) == count, 'RRT output continued after MPC CH10 latch'
            assert not mode_requests, 'Mission must not request automatic landing after CH10 cancellation'
            rc.channels[9] = 1000
            time.sleep(.3)
            assert phases[-1] == 'CANCELLED' and len(commands) == count
            print('PASS: actual MPC consumes RRT PositionCommand, publishes attitude, CH10 latches landing and cancels RRT output; no automatic resume')
            return
        wait_for(lambda: mode_requests, timeout=sum(result['duration']) + 15)
        assert len(mode_requests) == 1
        elapsed = rospy.Time.now().to_sec() - int(result['execution_start_ns']) / 1e9
        assert elapsed >= sum(result['duration']) + 2.
        wait_for(lambda: 'LANDING' in phases)
        assert 'DONE' not in phases
        extended.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        state.armed = False
        wait_for(lambda: 'DONE' in phases)
        assert len(goals) == 1
        assert phases == ['WAIT_START', 'PLANNING', 'FLYING', 'LAND_REQUESTED', 'LANDING', 'DONE'], phases
        timer.shutdown()
        rospy.signal_shutdown('test finished')
        print('PASS: manual takeoff gate, no start dwell, actual RRT trajectory, 2s arrival hover, AUTO.LAND, grounded/disarmed completion')
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for output in files:
            output.close()
        print('Mission test logs:', directory)


if __name__ == '__main__':
    main()
