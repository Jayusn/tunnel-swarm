#!/usr/bin/env python3
"""Control camera, YOLO, two rqt windows, and NUC-local crack capture/viewer."""
import contextlib
import ctypes
import ctypes.util
import datetime
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import uuid
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'deployment/wall_defect6/runtime'
STATE = RUNTIME / 'state.json'
TOPICS = {'camera': '/camera/color/image_raw', 'yolo': '/image_converter/output_video'}


def message(text):
    print(text, flush=True)


@contextlib.contextmanager
def locked():
    RUNTIME.mkdir(parents=True, exist_ok=True)
    with (RUNTIME / 'control.lock').open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        yield


def read_state():
    return json.loads(STATE.read_text()) if STATE.exists() else None


def write_state(state):
    temporary = STATE.with_suffix('.tmp')
    temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2))
    temporary.replace(STATE)


def identity(pid):
    try:
        base = Path('/proc') / str(pid)
        text = (base / 'stat').read_text()
        fields = text[text.rfind(')') + 2:].split()
        if fields[0] == 'Z':
            return None
        return {'pid': pid, 'start': fields[19], 'ppid': int(fields[1]),
                'command': (base / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')}
    except (FileNotFoundError, ProcessLookupError):
        return None


def alive(record):
    current = identity(record['pid'])
    return current is not None and current['start'] == record['start']


def owned_records(state):
    return list(state.get('processes', {}).values()) + state.get('children', [])


def descendants(pid):
    processes = []
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            try:
                record = identity(int(entry.name))
            except PermissionError:
                continue
            if record:
                processes.append(record)
    owned = {pid}
    while True:
        expanded = owned | {record['pid'] for record in processes if record['ppid'] in owned}
        if expanded == owned:
            return [record for record in processes if record['pid'] in owned and record['pid'] != pid]
        owned = expanded


def desktop_environment():
    # Select the logged-in physical desktop, including when invoked over SSH.
    sessions = subprocess.check_output(['loginctl', 'list-sessions', '--no-legend'], text=True)
    session_id = None
    for line in sessions.splitlines():
        fields = line.split()
        if len(fields) < 2 or fields[1] != str(os.getuid()):
            continue
        details = subprocess.check_output(['loginctl', 'show-session', fields[0],
                                          '-p', 'Active', '-p', 'Remote', '-p', 'Type'], text=True)
        if 'Active=yes' in details and 'Remote=no' in details and ('Type=x11' in details or 'Type=wayland' in details):
            session_id = fields[0]
            break
    if session_id is None:
        raise RuntimeError('NUC 没有已登录的本地图形桌面，请先在显示器上登录 oem 用户。')
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            command = (entry / 'comm').read_text().strip()
            if command not in ('gnome-shell', 'gnome-session-b', 'xfce4-session', 'plasmashell'):
                continue
            values = dict(item.decode(errors='replace').split('=', 1)
                          for item in (entry / 'environ').read_bytes().split(b'\0') if b'=' in item)
            if (values.get('XDG_SESSION_ID') and values.get('XDG_SESSION_ID') != session_id) or not values.get('DISPLAY'):
                continue
            environment = os.environ.copy()
            for key in ('DISPLAY', 'XAUTHORITY', 'DBUS_SESSION_BUS_ADDRESS', 'XDG_RUNTIME_DIR',
                        'WAYLAND_DISPLAY', 'XDG_SESSION_TYPE', 'XDG_CURRENT_DESKTOP'):
                if key in values:
                    environment[key] = values[key]
            subprocess.run(['xdpyinfo'], env=environment, stdout=subprocess.DEVNULL,
                           stderr=subprocess.PIPE, check=True, timeout=5)
            return environment
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    raise RuntimeError('无法连接 NUC 本机桌面，未修改 X11 的访问权限。')


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2
        return connection


def master():
    return xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'),
                                    transport=Transport())


def spawn(arguments, environment, log_path):
    with Path(log_path).open('ab') as log:
        process = subprocess.Popen(arguments, env=environment, cwd=ROOT, stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    record = identity(process.pid)
    if record is None:
        raise RuntimeError('进程启动后立即退出，请查看日志：' + str(log_path))
    return record


def check_camera_available():
    usb = subprocess.run(['lsusb', '-d', '2bdf:0001'], capture_output=True, text=True, timeout=5).stdout
    matches = re.findall(r'Bus (\d+) Device (\d+):', usb)
    if not matches:
        raise RuntimeError('未识别到海康 USB 相机，请检查 USB 连接。')
    for bus, device in matches:
        path = '/dev/bus/usb/{}/{}'.format(bus, device)
        result = subprocess.run(['fuser', path], capture_output=True, text=True, timeout=5)
        owners = []
        for value in re.findall(r'\d+', result.stdout):
            record = identity(int(value))
            if record:
                name = Path(record['command'].split(' ', 1)[0]).name
                owners.append('{} (PID {})'.format(name, value))
        if owners:
            raise RuntimeError('相机正在被 {} 占用，请先在该软件中断开相机连接。'.format('、'.join(owners)))


def current_run(run_id):
    with locked():
        state = read_state()
        return state if state and state['run_id'] == run_id and state['status'] != 'stopping' else None


def stop(expected_id=None):
    with locked():
        state = read_state()
        if state is None or (expected_id and state['run_id'] != expected_id):
            message('实验已停止。')
            return
        if state['status'] == 'stopping':
            message('实验正在停止，请稍候。')
            return
        state['status'] = 'stopping'
        pipeline = state.get('processes', {}).get('pipeline')
        if pipeline and alive(pipeline):
            # Refresh ownership before shutdown; an unrelated ROS master is excluded.
            records = {item['pid']: item for item in state.get('children', []) if alive(item)}
            records.update({item['pid']: item for item in descendants(pipeline['pid'])})
            state['children'] = list(records.values())
        write_state(state)
    message('正在停止本次实验的相机、YOLO、保存节点和图像窗口…')
    primary = list(state.get('processes', {}).items())
    primary.sort(key=lambda item: item[0] == 'pipeline')
    for _, record in primary:
        if alive(record):
            try:
                os.kill(record['pid'], signal.SIGINT)
            except ProcessLookupError:
                pass
    records = owned_records(state)
    for sig, seconds in ((None, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        pending = [record for record in records if alive(record)]
        if sig:
            for record in pending:
                try:
                    os.kill(record['pid'], sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + seconds
        while pending and time.monotonic() < deadline:
            time.sleep(.15)
            pending = [record for record in pending if alive(record)]
        if not pending:
            break
    if pending:
        raise RuntimeError('仍有实验进程未退出：' + ', '.join(str(item['pid']) for item in pending))
    with locked():
        current = read_state()
        if current and current['run_id'] == state['run_id']:
            state['status'] = 'stopped'
            (RUNTIME / 'last_run.json').write_text(json.dumps(state, ensure_ascii=False, indent=2))
            STATE.unlink()
    message('已停止。实验日志保留在：' + state['log_dir'])


def task_windows(processes, environment, names=None):
    listing = subprocess.check_output(['xprop', '-root', '_NET_CLIENT_LIST'], env=environment,
                                      text=True, timeout=5)
    names = TOPICS if names is None else names
    by_pid = {record['pid']: name for name, record in processes.items() if name in names}
    windows = {}
    for value in re.findall(r'0x[0-9a-fA-F]+', listing):
        props = subprocess.check_output(['xprop', '-id', value, '_NET_WM_PID'],
                                        env=environment, text=True, timeout=3)
        match = re.search(r'= (\d+)', props)
        if match and int(match.group(1)) in by_pid:
            windows[by_pid[int(match.group(1))]] = int(value, 16)
    return windows


def arrange_windows(windows, environment):
    from experiment_window_layout import arrange
    return arrange(windows, environment)


def refit_windows():
    with locked():
        state = read_state()
        if not state:
            raise RuntimeError('实验未启动，请先运行一键启动脚本。')
        environment = desktop_environment()
        names = ('camera', 'yolo', 'pixels')
        windows = task_windows(state['processes'], environment, names)
        result = arrange_windows(windows, environment)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        message('窗口已按桌面可用区域重新排布。')


def wait_for_images(run_id, timeout=40):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = current_run(run_id)
        if state is None:
            return False
        if not alive(state['processes']['pipeline']):
            raise RuntimeError('相机/YOLO 启动退出，请查看 camera_yolo.log。')
        try:
            if master().getPid('/wall_defect_experiment_control')[0] == 1:
                break
        except (OSError, xmlrpc.client.Error):
            pass
        time.sleep(.2)
    else:
        raise RuntimeError('ROS master 启动超时。')
    import rospy
    from sensor_msgs.msg import Image
    rospy.init_node('wall_defect_experiment_check', anonymous=True, disable_signals=True)
    received = {name: [] for name in TOPICS}

    def callback(name):
        return lambda _: received[name].append(time.monotonic())

    subscriptions = [rospy.Subscriber(topic, Image, callback(name), queue_size=1, buff_size=2**25)
                     for name, topic in TOPICS.items()]
    try:
        while time.monotonic() < deadline:
            state = current_run(run_id)
            if state is None:
                return False
            if not alive(state['processes']['pipeline']):
                raise RuntimeError('相机/YOLO 进程已退出，请查看 camera_yolo.log。')
            now = time.monotonic()
            if all(len(times) >= 3 and now - times[-1] < 2 for times in received.values()):
                return True
            time.sleep(.2)
    finally:
        for subscription in subscriptions:
            subscription.unregister()
        rospy.signal_shutdown('Experiment startup check complete')
    raise RuntimeError('40 秒内没有收到连续相机原图和 YOLO 结果，请检查 USB 相机与日志。')


def start(arguments):
    run_id = None
    try:
        with locked():
            previous = read_state()
            if previous:
                if any(alive(record) for record in owned_records(previous)):
                    message('实验' + ('正在停止，请稍后再启动。' if previous['status'] == 'stopping' else '已启动或正在启动，不会重复打开。'))
                    message('日志目录：' + previous['log_dir'])
                    return
                STATE.unlink()
            binary = ROOT / 'devel/lib/yolo_detector/rosnode_yolo'
            camera_binary = Path('/opt/ros/noetic/lib/realsense2_camera/realsense2_camera_node')
            if not binary.is_file():
                raise RuntimeError('未找到相机或 YOLO 可执行文件，请先编译当前工作空间。')
            environment = desktop_environment()
            capture_enabled = environment.get('CRACK_CAPTURE_ENABLED', '1') != '0'
            capture_root = Path(environment.get('CRACK_CAPTURE_DIR',
                                                str(ROOT / 'captures/crack_live'))).expanduser().resolve()
            if capture_enabled:
                for filename in ('crack_capture_ros.py', 'crack_pixel_viewer.py',
                                 'crack_live_common.py', 'crack_live_store.py'):
                    if not (ROOT / 'scripts' / filename).is_file():
                        raise RuntimeError('缺少在线保存文件，请完整同步 scripts：' + filename)
                check = subprocess.run(
                    [sys.executable, '-c',
                     'import cv2,numpy,rospy; from PyQt5 import QtWidgets; from target_ros_msgs.msg import BoundingBoxes'],
                    env=environment, capture_output=True, text=True, timeout=15)
                if check.returncode:
                    raise RuntimeError('在线保存/像素查看依赖缺失：' + check.stderr.strip())
            # RealSense driver reports device availability during startup.
            try:
                _, _, system_state = master().getSystemState('/wall_defect_experiment_control')
                nodes = {node for section in system_state for _, names in section for node in names}
            except (OSError, xmlrpc.client.Error):
                nodes = set()
            if nodes & {'/camera/realsense2_camera', '/rosnode_yolo'}:
                raise RuntimeError('ROS 中已有其他方式启动的相机/YOLO 同名节点，请先关闭原节点。')
            run_id = uuid.uuid4().hex
            log_dir = ROOT / 'logs' / ('experiment_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + run_id[:6])
            log_dir.mkdir(parents=True)
            state = {'run_id': run_id, 'status': 'starting', 'log_dir': str(log_dir),
                     'started_at': datetime.datetime.now().astimezone().isoformat(),
                     'processes': {}, 'children': [], 'capture_enabled': capture_enabled,
                     'capture_dir': str(capture_root) if capture_enabled else None}
            state['processes']['pipeline'] = spawn(['roslaunch', 'yolo_detector', 'realsense_yolo.launch'] + arguments,
                                                    environment, log_dir / 'camera_yolo.log')
            write_state(state)
        message('正在启动RealSense 相机和 YOLO，等待两路连续图像…')
        if not wait_for_images(run_id):
            message('启动已取消。')
            return
        with locked():
            state = read_state()
            if not state or state['run_id'] != run_id or state['status'] == 'stopping':
                message('启动已取消。')
                return
            for name, topic in TOPICS.items():
                state['processes'][name] = spawn(['rosrun', 'rqt_image_view', 'rqt_image_view', topic],
                                                 environment, Path(state['log_dir']) / ('rqt_' + name + '.log'))
                # Save immediately so Stop can also interrupt GUI startup.
                write_state(state)
            if state.get('capture_enabled'):
                auxiliary = [
                    ('recorder', 'crack_capture_ros.py', '--output'),
                    ('pixels', 'crack_pixel_viewer.py', '--root'),
                ]
                for name, script, option in auxiliary:
                    state['processes'][name] = spawn(
                        [sys.executable, str(ROOT / 'scripts' / script), option, state['capture_dir']] + (['--tiled'] if name == 'pixels' else []),
                        environment, Path(state['log_dir']) / (name + '.log'))
                    write_state(state)
            state['children'] = descendants(state['processes']['pipeline']['pid'])
            write_state(state)
        message('相机与 YOLO 已出图，正在打开 NUC 本机图像窗口…')
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            state = current_run(run_id)
            if state is None:
                message('启动已取消。')
                return
            if not all(alive(record) for record in state['processes'].values()):
                raise RuntimeError('实验或 rqt 窗口启动失败，请查看日志。')
            expected_windows = ('camera', 'yolo', 'pixels') if state.get('capture_enabled') else tuple(TOPICS)
            windows = task_windows(state['processes'], environment, expected_windows)
            capture_ready = True
            if state.get('capture_enabled'):
                try:
                    status = json.loads((Path(state['capture_dir']) / 'status.json').read_text())
                    capture_ready = (status.get('running') and
                                     status.get('pid') == state['processes']['recorder']['pid'] and
                                     time.time() - (status.get('last_matched_at') or 0) < 3)
                except (OSError, ValueError):
                    capture_ready = False
            if len(windows) == len(expected_windows) and capture_ready:
                with locked():
                    current = read_state()
                    if not current or current['run_id'] != run_id or current['status'] == 'stopping':
                        message('启动已取消。')
                        return
                    arrange_windows(windows, environment)
                    current['status'] = 'running'
                    current['children'] = descendants(current['processes']['pipeline']['pid'])
                    write_state(current)
                if state.get('capture_enabled'):
                    message('启动完成：左侧上下为相机原图/检测结果，右侧为保存后的像素查看。')
                    message('裂缝自动保存和本机像素窗口已启动，保存目录：' + state['capture_dir'])
                else:
                    message('启动完成：左侧为相机原图，右侧为 YOLO 检测结果。')
                message('日志目录：' + state['log_dir'])
                return
            time.sleep(.3)
        raise RuntimeError('图像窗口或保存节点就绪超时，请查看 rqt、recorder、pixels 日志。')
    except BaseException:
        if run_id:
            stop(run_id)
        raise


def main():
    action = sys.argv[1] if len(sys.argv) > 1 else 'status'
    try:
        if action == 'start':
            start(sys.argv[2:])
        elif action == 'stop':
            stop()
        elif action == 'layout':
            refit_windows()
        elif action == 'status':
            with locked():
                state = read_state()
                print(json.dumps(state, ensure_ascii=False, indent=2) if state else '实验已停止。')
        else:
            raise RuntimeError('用法：experiment_control.py start [roslaunch 参数] | stop | status | layout')
    except KeyboardInterrupt:
        message('操作被中断；可运行 stop_experiment.sh 停止实验。')
        return 130
    except Exception as error:
        message('操作失败：' + str(error))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
