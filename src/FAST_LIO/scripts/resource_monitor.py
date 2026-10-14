#!/usr/bin/env python3
"""Read-only 1 Hz /proc sampling; counters are cumulative for offline differencing."""
import json
import os
from pathlib import Path
import time

import rospy
import rospkg


def read_text(path):
    try:
        return Path(path).read_text()
    except (OSError, UnicodeError):
        return None


def task_stat(path):
    raw = read_text(path)
    if raw is None:
        return None
    end = raw.rfind(')')
    fields = raw[end + 2:].split()
    return dict(name=raw[raw.find('(') + 1:end], state=fields[0],
                minflt=int(fields[7]), majflt=int(fields[9]),
                utime=int(fields[11]), stime=int(fields[12]),
                start_ticks=int(fields[19]), rss_pages=int(fields[21]))


def sample():
    result = dict(wall_time=time.time(), monotonic=time.monotonic(),
                  cpu=read_text('/proc/stat'), memory=read_text('/proc/meminfo'),
                  vm=read_text('/proc/vmstat'), disks=read_text('/proc/diskstats'),
                  pressure={k: read_text('/proc/pressure/' + k) for k in ('cpu', 'memory', 'io')},
                  processes={})
    for entry in os.scandir('/proc'):
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            base = Path(entry.path)
            stat = task_stat(base / 'stat')
            if stat is None:
                continue
            # Store only executable/name for identification, never command-line secrets.
            cmd = read_text(base / 'cmdline') or ''
            stat['executable'] = cmd.split('\0')[0]
            stat['io'] = read_text(base / 'io')
            if any(x in cmd for x in ('/fastlio_mapping', '/mapping_node', '/diff_planner_node', '/rosbag/record')):
                stat['threads'] = {}
                for thread in (base / 'task').iterdir():
                    info = task_stat(thread / 'stat')
                    if info:
                        info['wchan'] = read_text(thread / 'wchan')
                        stat['threads'][thread.name] = info
            result['processes'][entry.name] = stat
        except (OSError, ValueError, IndexError):
            # A process/thread can exit while /proc is read.
            continue
    return result


def main():
    rospy.init_node('resource_monitor')
    directory = Path(rospkg.get_log_dir()) / rospy.get_param('/run_id', '')
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / ('resources_%d.jsonl' % os.getpid())
    rospy.loginfo('Resource log: %s', path)
    with path.open('w', buffering=65536) as output:
        output.write(json.dumps(dict(type='metadata', clock_ticks=os.sysconf('SC_CLK_TCK'),
                                     page_size=os.sysconf('SC_PAGE_SIZE'), cpu_count=os.cpu_count(),
                                     interval_s=1)) + '\n')
        deadline = time.monotonic()
        count = 0
        while not rospy.is_shutdown():
            started = time.monotonic()
            data = sample()
            data['sample_ms'] = (time.monotonic() - started) * 1000
            output.write(json.dumps(data, separators=(',', ':')) + '\n')
            count += 1
            if count % 5 == 0:
                output.flush()
            deadline += 1
            time.sleep(max(0, min(1, deadline - time.monotonic())))
            if time.monotonic() > deadline + 1:
                deadline = time.monotonic()


if __name__ == '__main__':
    main()
