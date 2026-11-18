#!/usr/bin/env bash
# Start a fresh FUEL map for each flight; manual takeoff remains under RC control.
set -e
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
用法：rosrun rrt_minco_adapter start_flight.sh [roslaunch参数...]
每次启动全新的FUEL占据地图、RRT指令发布器和点到点任务。
先退出上一轮任务及建图节点，重新扫描并调用/UAV0/rrt/freeze_map。
从地面(0.01,0.01,0)起飞，遥控器解锁并切入OFFBOARD，
现有控制器须设置offboard_takeoff_height:=0.5，上升至(0.01,0.01,0.5)。
起点稳定2秒后从真实里程计位置规划，50Hz发布/UAV0/planning/pos_cmd；
到达(3,3,0.5)悬停2秒后请求AUTO.LAND。脚本不解锁，也不启动控制器或雷达。
不要同时运行其他RRT/FUEL/Diff飞行指令发布者。
不复用上一轮地图；脚本保持运行，另开终端冻结本轮地图后再起飞。
HELP
  exit 0
fi
exec roslaunch rrt_minco_adapter point_to_point_mission.launch "$@"
