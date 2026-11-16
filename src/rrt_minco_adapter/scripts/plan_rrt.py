#!/usr/bin/env python3
"""Portable, planning-only RRT/MINCO API. No ROS or FCU commands."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'rrt_core'))
from local_rrt_core import Scene,plan
from trajectory_backend import build_trajectory,backend_stats


def solve(data,start=None,goal=None,speed=.6,acceleration=.8,clearance=.3,reserve=.1):
    data=dict(data);start=data['start'] if start is None else start;goal=data['goal'] if goal is None else goal
    scene=Scene(data,clearance=clearance+reserve)
    began=time.monotonic()
    path,stats=plan(scene,start,goal,seed=7,iterations=12000,timeout=8,optimization_time=.5)
    traj=build_trajectory(path,scene,speed,acceleration,time_optimization=False,time_evaluations=0,
        pair_search=False,terminal_braking=False,start_turn_braking=False,position_length_weight=1.,
        position_speed_weight=1.,position_ftol=1e-8,compact_constraints=True,improve_initial=True)
    coeff=np.transpose(traj.coefficients,(2,0,1))
    # 共用 SUPER JSON 的物理局部时间降幂系数格式；此处为二维轨迹。
    x=(coeff[0]/traj.durations[:,None]**np.arange(6)).reshape(-1,6)[:,::-1].ravel().tolist()
    y=(coeff[1]/traj.durations[:,None]**np.arange(6)).reshape(-1,6)[:,::-1].ravel().tolist()
    result=dict(frame_id='world',dimensions=2,order=5,duration=traj.durations.tolist(),coef_x=x,coef_y=y,
        start=list(start),goal=list(goal),path_m=path,planning_clearance_m=scene.margin,
        evaluation_clearance_m=clearance,tracking_margin_m=reserve,front=stats,backend=backend_stats(traj),
        total_wall_s=time.monotonic()-began)
    return traj,result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,default=ROOT/'config/example_scene.json')
    p.add_argument('--output',type=Path,default=ROOT/'runtime/rrt_trajectory.json')
    p.add_argument('--speed',type=float,default=.6);p.add_argument('--acceleration',type=float,default=.8)
    p.add_argument('--clearance',type=float,default=.3);p.add_argument('--tracking-margin',type=float,default=.1)
    a=p.parse_args()
    if not np.isfinite(a.tracking_margin) or a.tracking_margin<0:p.error('tracking-margin must be finite and nonnegative')
    traj,r=solve(json.loads(a.scene.read_text()),speed=a.speed,acceleration=a.acceleration,clearance=a.clearance,reserve=a.tracking_margin)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(r,indent=2))
    print('轨迹文件：',a.output);print('参考时间 %.3fs，长度 %.3fm；仅规划，没有控制无人机。'%(traj.total_time,traj.s[-1]))
if __name__=='__main__':main()
