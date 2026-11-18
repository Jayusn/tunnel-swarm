#!/usr/bin/env bash
# Use the existing surveyed FUEL map; manual takeoff remains under RC control.
set -e
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
用法：rosrun rrt_minco_adapter start_flight.sh [roslaunch参数...]
默认复用正在运行且已冻结的FUEL地图，启动RRT指令发布器和点到点任务。
起点(0.01,0.01,0.5)，目标(3,3,0.5)。遥控器起飞并切入OFFBOARD，
起点稳定2秒后从真实里程计位置规划，50Hz发布/UAV0/planning/pos_cmd；
到点悬停3秒后请求AUTO.LAND。脚本不解锁，也不启动控制器或雷达。
不要同时运行其他RRT/FUEL/Diff飞行指令发布者。
可用start_mapper:=true启动一套新FUEL地图；不要与已有地图节点重复启动。
HELP
  exit 0
fi
exec roslaunch rrt_minco_adapter point_to_point_mission.launch start_mapper:=false "$@"
