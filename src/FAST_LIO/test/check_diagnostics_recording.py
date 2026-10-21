#!/usr/bin/env python3
"""Isolated synthetic ROS smoke test. No hardware nodes or control commands.
Run after sourcing devel/setup.bash; output stays outside the repository.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--sensor-wrapper', action='store_true')
    args = parser.parse_args()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ['ROS_LOG_DIR'] = str(out / 'roslog')
    processes = []
    logs = []

    def start(name, command):
        log = (out / (name + '.log')).open('w')
        logs.append(log)
        p = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True)
        processes.append(p)
        return p

    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
                raise RuntimeError('Test subprocess failed to close cleanly')

    # Sample RSS of the actual record process, not the roslaunch parent.
    def recorder_rss(group):
        for path in Path('/proc').glob('[0-9]*'):
            try:
                # Only inspect children belonging to this isolated ROS master.
                environment = (path / 'environ').read_bytes().split(b'\0')
                if ('ROS_MASTER_URI='+os.environ['ROS_MASTER_URI']).encode() not in environment:
                    continue
                command = (path / 'cmdline').read_bytes()
                if b'/rosbag/record\x00' not in command:
                    continue
                for line in (path / 'status').read_text().splitlines():
                    if line.startswith('VmRSS:'):
                        return int(line.split()[1])
            except (OSError, ProcessLookupError):
                pass
        return 0

    try:
        start('master', ['roscore', '-p', str(port)])
        deadline = time.monotonic()+10
        while True:
            try:
                xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/smoke')
                break
            except OSError:
                if time.monotonic()>deadline:
                    raise RuntimeError('isolated master did not start')
                time.sleep(.1)
        import rospy
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String
        from livox_ros_driver2.msg import CustomMsg, CustomPoint
        from fast_lio.msg import StateDiagnostics
        rospy.init_node('synthetic_diagnostics_test', disable_signals=True)
        diagnostics = []
        rospy.Subscriber('/UAV1/fast_lio/state_diagnostics', StateDiagnostics,
                         diagnostics.append, queue_size=20)
        if args.sensor_wrapper:
            workspace = Path(__file__).resolve().parents[3]
            recorder = start('sensor_wrapper', ['bash', str(workspace/'shfiles/run_uav1_sensor_stack.sh'), 'fastlio'])
            rospy.wait_for_message('/UAV1/lio_recording_started', String, timeout=15)
            mapping = recorder
            import re
            match = re.search(r'原始点云/诊断录包：(.*)', (out/'sensor_wrapper.log').read_text())
            assert match, 'recording directory was not announced'
            bag_dir = Path(match.group(1).strip())
            (bag_dir/'SYNTHETIC_TEST.txt').write_text('Synthetic inputs; not a flight.\n')
            (out/'raw_record_dir.txt').write_text(str(bag_dir)+'\n')
        else:
            recorder = start('recorder', ['roslaunch', 'fast_lio', 'record_lio_diagnostics.launch',
                                         'bag_prefix:='+str(out/'raw')])
            rospy.wait_for_message('/UAV1/lio_recording_started', String, timeout=10)
            mapping = start('mapping', ['roslaunch', 'fast_lio', 'mapping_mid360.launch',
                                       'rviz:=false', 'performance_logging:=false'])
            bag_dir = out
        imu = rospy.Publisher('/UAV1/livox/imu', Imu, queue_size=100)
        px4 = rospy.Publisher('/UAV1/mavros/imu/data_raw', Imu, queue_size=100)
        cloud = rospy.Publisher('/UAV1/livox/lidar', CustomMsg, queue_size=5)
        deadline = time.monotonic()+15
        while imu.get_num_connections()<2 or cloud.get_num_connections()<2:
            if time.monotonic()>deadline:
                raise RuntimeError('recorder/mapping input subscribers did not connect')
            time.sleep(.1)
        # Static floor and two walls. Inputs imitate already body-rotated driver points.
        points = []
        for i in range(30):
            for j in range(30):
                a=-2.+4.*i/29.; b=-2.+4.*j/29.
                for xyz in [(a,b,-1.), (2.,a,b), (a,2.,b)]:
                    p=CustomPoint(); p.x,p.y,p.z=xyz
                    p.reflectivity=30; p.tag=0; p.line=i%4
                    points.append(p)
        for i,p in enumerate(points):
            p.offset_time=int(i*100000000/len(points))
        rate=rospy.Rate(200)
        rss=[]
        for i in range(2000):
            now=rospy.Time.now()
            m=Imu(); m.header.stamp=now; m.header.frame_id='UAV1/livox_frame'
            m.linear_acceleration.x=-math.sin(math.radians(15))
            m.linear_acceleration.z=math.cos(math.radians(15))
            imu.publish(m)
            flight_imu=Imu(); flight_imu.header.stamp=now
            flight_imu.linear_acceleration.z=9.81; px4.publish(flight_imu)
            if i%20==0:
                c=CustomMsg(); c.header.stamp=now-rospy.Duration(.1)
                c.header.frame_id='UAV1/body'; c.timebase=c.header.stamp.to_nsec()
                c.point_num=len(points); c.points=points
                cloud.publish(c)
            if i%100==0:
                rss.append(recorder_rss(recorder.pid))
            rate.sleep()
        stop(mapping)
        stop(recorder)
        assert len(diagnostics)>20, len(diagnostics)
        valid=[d for d in diagnostics if d.final_measurement_valid]
        assert valid, 'no valid synthetic laser matches'
        for d in valid:
            assert d.downsampled_points >= d.matched_points > 0
            assert d.iteration_count >= d.valid_iteration_count > 0
            assert d.covariance_updated
            assert 0 <= d.residual_mean_abs_m <= d.residual_rms_m+1e-6
            for q in [d.attitude_before,d.attitude_after]:
                assert abs(sum(v*v for v in q)-1)<1e-8
            for p in [d.covariance_before,d.covariance_after]:
                assert len(p)==529 and all(math.isfinite(v) for v in p)
        import rosbag
        bags=sorted(bag_dir.glob('raw*.bag'))
        assert bags and not list(bag_dir.glob('*.active'))
        if args.sensor_wrapper:
            assert (bag_dir/'params_end.yaml').is_file()
            assert (bag_dir/'binary.sha256').is_file()
        assert max(rss)>0, 'recorder RSS was not sampled'
        counts={}
        for path in bags:
            with rosbag.Bag(str(path)) as bag:
                for topic,msg,_ in bag.read_messages():
                    counts[topic]=counts.get(topic,0)+1
                    if topic=='/UAV1/livox/lidar':
                        assert msg._type=='livox_ros_driver2/CustomMsg'
                        assert msg.point_num==len(points)
                        assert msg.points[-1].offset_time==points[-1].offset_time
        for topic in ['/UAV1/livox/lidar','/UAV1/livox/imu',
                      '/UAV1/mavros/imu/data_raw','/UAV1/fast_lio/state_diagnostics']:
            assert counts.get(topic,0)>0, topic
        import statistics
        report={'synthetic_only':True, 'diagnostics_received':len(diagnostics),
                'valid_updates':len(valid), 'topic_counts':counts,
                'bag_bytes':sum(p.stat().st_size for p in bags),
                'recorder_sampled_max_rss_kib':max(rss),
                'update_ms_median':statistics.median(d.update_wall_ms for d in valid),
                'update_ms_max':max(d.update_wall_ms for d in valid)}
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        for p in reversed(processes):
            stop(p)
        for log in logs:
            log.close()


if __name__=='__main__':
    main()
