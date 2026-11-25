#!/usr/bin/env bash
# 仅启动 MID360、FAST-LIO、FUEL 占据建图和 RViz。
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
用法：bash ~/match_ws/shfiles/start_rrt_mapping.sh
依次启动雷达、FAST-LIO、FUEL 三维占据地图和 RViz，已有雷达/定位会复用。
不启动控制器、飞行任务、MAVROS 或录包；保持无人机未解锁。
从原起飞点附近静置启动，看到“扫描建图已启动”后再缓慢移动补扫。
扫描完成后另开 ROS 终端冻结、保存地图；本脚本不自动冻结或保存。
Ctrl+C 结束本脚本启动的进程；原来已运行的雷达/定位保留。
HELP
  exit 0
fi
if (( $# )); then echo '不接受参数；查看用法：start_rrt_mapping.sh --help' >&2; exit 2; fi
source "${SCRIPT_DIR}/../devel/setup.bash"
source "${SCRIPT_DIR}/configure_dual_uav_ros_network.sh"
configure_dual_uav_ros_network uav0

children=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${children[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() {
  echo "[启动] $*"
  setsid "$@" &
  children+=("$!")
}
wait_topic() {
  echo "[等待数据] $1"
  if ! timeout 60 rostopic echo -n 1 "${1}/header" >/dev/null; then
    echo "等待话题超时：$1；未继续启动后续节点，请检查对应终端输出。" >&2
    exit 1
  fi
}
nodes="$(timeout 5 rosnode list 2>/dev/null || true)"
has_node() { [[ $'\n'"${nodes}"$'\n' == *$'\n'"$1"$'\n'* ]]; }
# 沿用实飞入口的重复启动检查，避免踢掉旧任务或复用已冻结地图。
for node in /rrt_saved_map /rrt_fuel_map /rrt_cloud_scene /rrt_grid_rviz /rrt_minco_adapter /rrt_point_to_point_mission /simple_controller /mpc_controller_node /UAV0/landing_setpoint_arbiter; do
  if has_node "$node"; then
    echo "请先在地面退出旧入口（仍有${node}），再启动扫描脚本。" >&2
    exit 1
  fi
done
if has_node /UAV0_livox_imu_to_body && ! has_node /UAV0/laserMapping; then
  echo '仍有旧 FAST-LIO 的 IMU 转换节点，请先结束旧 FAST-LIO 启动终端。' >&2
  exit 1
fi
if [[ -z "$nodes" ]]; then
  start roscore
  for ((attempt=0; attempt<30; attempt++)); do
    if nodes="$(timeout 2 rosnode list 2>/dev/null)" && [[ -n "$nodes" ]]; then break; fi
    sleep 1
  done
  [[ -n "$nodes" ]] || { echo 'ROS master启动失败'; exit 1; }
fi
if ! has_node /UAV0/livox_lidar_publisher2; then
  start roslaunch livox_ros_driver2 msg_MID360.launch vehicle_ns:=UAV0 msg_frame_id:=UAV0/livox_frame publish_freq:=10.0
fi
wait_topic /UAV0/livox/lidar
wait_topic /UAV0/livox/imu
if ! has_node /UAV0/laserMapping; then
  start roslaunch fast_lio mapping_mid360.launch vehicle_ns:=UAV0 rviz:=false
fi
wait_topic /UAV0/fast_lio/Odometry
wait_topic /UAV0/fast_lio/cloud_registered
start roslaunch rrt_minco_adapter grid_map_view.launch
wait_topic /UAV0/rrt/fuel/occupancy_all
wait_topic /UAV0/rrt/voxel_map
cat <<'HELP'
扫描建图已启动。保持本终端运行，观察 RViz 三维方块地图，缓慢移动补扫遮挡区域。
确认地图完整、人员残影已处理后，另开终端执行：
  source ~/match_ws/devel/setup.bash
  source ~/match_ws/shfiles/configure_dual_uav_ros_network.sh
  configure_dual_uav_ros_network uav0
  rosservice call /UAV0/rrt/freeze_map "{}"
  rosparam set /rrt_cloud_scene/save_path "$HOME/rrt_maps/site.json"
  rosservice call /UAV0/rrt/save_map "{}"
save_map 返回 success: True 才表示已保存；已有同名文件时请换文件名。
保存成功后 Ctrl+C 结束扫描。本脚本沿用现有地图过滤，不自动识别人。
HELP
wait
