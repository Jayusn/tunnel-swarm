#!/usr/bin/env python3
"""Compare fixed-height RRT and SUPER on one frozen scene; no flight output."""
import json
from pathlib import Path
import time
import sys
import rospkg
import numpy as np
import rospy
from std_msgs.msg import String
sys.path.insert(0, str(Path(rospkg.RosPack().get_path('rrt_minco_adapter'))/'scripts'))
from plan_rrt import solve as solve_rrt
from plan_super import solve as solve_super


def main():
    rospy.init_node('compare_rrt_super', anonymous=True)
    source = rospy.get_param('~scene_file', '')
    scene = (json.loads(Path(source).expanduser().read_text()) if source else
             json.loads(rospy.wait_for_message('/UAV0/rrt/scene', String, timeout=60).data))
    scene['boundary_clearance'] = 0.
    start = rospy.get_param('~start', [0., 0.]); goal = rospy.get_param('~goal', [4., 5.])
    options = {key: float(rospy.get_param('~'+key, value)) for key, value in
               [('speed', .2), ('acceleration', .8), ('clearance', .2), ('reserve', .1)]}
    height = float(rospy.get_param('~height', .5))
    output = Path(rospy.get_param('~output_dir', str(Path.home()/'.ros/planner_comparison'/time.strftime('%Y%m%d_%H%M%S')))).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    (output/'scene.json').write_text(json.dumps(scene, indent=2))
    rows = []
    for trial in range(int(rospy.get_param('~trials', 1))):
        for name, solve in [('rrt', solve_rrt), ('super', solve_super)]:
            row = dict(planner=name, trial=trial, start=start, goal=goal, height=height, **options)
            began = time.monotonic()
            try:
                trajectory, result = solve(scene, start, goal, **options, **({'height':height} if name=='super' else {}))
                times = np.linspace(0, trajectory.total_time, max(2, int(trajectory.total_time/.02)+1))
                pos = np.array([trajectory.sample(t)[0] for t in times])
                jerk2 = np.array([np.sum(trajectory.derivative(t, 3)**2) for t in times])
                row.update(success=True, duration_s=trajectory.total_time,
                           length_m=float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()),
                           jerk_squared_integral=float(np.trapz(jerk2, times)),
                           total_wall_s=result['total_wall_s'], native_wall_s=result.get('native_wall_s'),
                           time_stretch=result.get('time_stretch', 1.))
                result['planner'] = name
                (output/('%s_%d.json'%(name,trial))).write_text(json.dumps(result, indent=2))
            except Exception as error:
                row.update(success=False, error=str(error), total_wall_s=time.monotonic()-began)
            rows.append(row)
            rospy.loginfo('%s', json.dumps(row, ensure_ascii=False))
            (output/'summary.json').write_text(json.dumps(rows, indent=2, ensure_ascii=False))
    rospy.loginfo('对比完成：%s；没有发布飞行指令', output)


if __name__ == '__main__':
    main()
