#!/usr/bin/env bash
# Start fresh mapping or load an explicitly supplied map; takeoff remains under RC control.
set -e
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
用法：rosrun rrt_minco_adapter start_flight.sh [roslaunch参数...]
每次启动全新的FUEL占据地图、RRT指令发布器和点到点任务。
先退出上一轮任务及建图节点，重新扫描并调用/UAV0/rrt/freeze_map。
从地面(0,0,0)起飞，遥控器解锁并切入OFFBOARD，
使用autotrans_controller.launch；CH8低位起飞至0.5m，再拨高位允许轨迹跟踪。
CH8高位且起点条件满足后立即从真实里程计位置规划，50Hz发布/UAV0/planning/pos_cmd；
到达(4,5,0.5)满足终点条件后立即请求MPC降落（参考0.2m/s，PX4收尾）。脚本不解锁，也不启动控制器或雷达。
CH10先低后高触发MPC降落，RRT任务取消并停止指令；需落地后重启下一轮。
不要同时运行简单控制器、其他RRT/FUEL/Diff飞行指令发布者。
默认新建地图；传入 map_file:=/绝对路径/site.json 时复用保存地图。
旧地图模式下 freeze_map 服务执行本次开机定位匹配，成功后发布旧场景。
详细步骤见 README 的断电后复用地图章节；先做未解锁对齐验证。
HELP
  exit 0
fi
exec roslaunch rrt_minco_adapter point_to_point_mission.launch "$@"
