#!/usr/bin/env bash
# 一键启动实飞链路；解锁和OFFBOARD仍由遥控器操作。
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" ]]; then
  echo '用法：bash ~/match_ws/shfiles/start_rrt_flight_all.sh'
  echo '启动基础链路、全新FUEL地图、RRT任务、控制器、仲裁器和RViz。'
  echo '先在地面退出上一轮任务/建图/控制器；已有雷达、定位、MAVROS和仲裁器保留。'
  echo '扫描后另开终端冻结地图；遥控解锁并切入OFFBOARD后自动执行任务。'
  echo '落地解除武装后Ctrl+C结束本脚本启动的节点；复用节点不停止。'
  exit 0
fi
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
  if ! timeout 60 rostopic echo -n 1 "$1" >/dev/null; then
    echo "等待话题超时：$1；未继续启动后续节点。" >&2
    exit 1
  fi
}
nodes="$(timeout 5 rosnode list 2>/dev/null || true)"
has_node() { [[ $'\n'"${nodes}"$'\n' == *$'\n'"$1"$'\n'* ]]; }
# 避免旧地图、旧任务或旧控制器被同名新节点替换。
for node in /rrt_fuel_map /rrt_cloud_scene /rrt_minco_adapter /rrt_point_to_point_mission /simple_controller /rrt_grid_rviz; do
  if has_node "$node"; then
    echo "请先在地面退出旧入口（仍有${node}），再启动本脚本。" >&2
    exit 1
  fi
done
if [[ -z "$nodes" ]]; then
  start roscore
  for attempt in {1..30}; do
    if nodes="$(timeout 2 rosnode list 2>/dev/null)"; then break; fi
    sleep 1
  done
  [[ -n "$nodes" ]] || { echo 'ROS master启动失败'; exit 1; }
fi
if ! has_node /UAV0/mavros; then start bash "${SCRIPT_DIR}/run.sh"; fi
if ! has_node /UAV0/livox_lidar_publisher2; then
  start roslaunch livox_ros_driver2 msg_MID360.launch vehicle_ns:=UAV0 msg_frame_id:=UAV0/livox_frame publish_freq:=10.0
fi
wait_topic /UAV0/livox/lidar
wait_topic /UAV0/livox/imu
if ! has_node /UAV0/laserMapping; then
  start roslaunch fast_lio mapping_mid360.launch vehicle_ns:=UAV0 odom_topic:=/UAV0/fast_lio/Odometry high_freq_odom_topic:=/UAV0/fast_lio/Odom_high_freq rviz:=false
fi
wait_topic /UAV0/fast_lio/Odom_high_freq
wait_topic /UAV0/fast_lio/cloud_registered
if ! has_node /UAV0_fastlio_laser_transfer; then
  start python3 "${SCRIPT_DIR}/../src/cxr_ego_ctrl/src/laser_mid360.py" iris 0 fastlio off _odom_topic:=/UAV0/fast_lio/Odom_high_freq
fi
python3 "${SCRIPT_DIR}/wait_rrt_localization.py"
if ! has_node /UAV0/landing_setpoint_arbiter; then
  start rosrun precision_landing landing_setpoint_arbiter_node __ns:=/UAV0 __name:=landing_setpoint_arbiter \
    _trigger_topic:=/UAV0/need_to_land \
    _rc_land_active_topic:=/UAV0/mission/rc_land_active \
    _controller_position_input:=/UAV0/control/position_setpoint \
    _controller_attitude_input:=/UAV0/control/attitude_setpoint \
    _landing_position_input:=/UAV0/landing/setpoint_raw/local \
    _mavros_position_output:=/UAV0/mavros/setpoint_raw/local \
    _mavros_attitude_output:=/UAV0/mavros/setpoint_raw/attitude \
    _active_topic:=/UAV0/landing/control_active \
    _owner_topic:=/UAV0/landing/control_owner
fi
start roslaunch exploration_control simple_controller.launch offboard_takeoff_height:=0.5 planner_enable_height:=0.45
start rosrun rrt_minco_adapter start_flight.sh
start roslaunch rrt_minco_adapter grid_map_view.launch start_mapper:=false
cat <<'HELP'
链路已启动，请保持终端运行并检查各节点输出。
本轮重新扫描后，另开已加载ROS环境的终端执行：
  rosservice call /UAV0/rrt/freeze_map "{}"
确认地图、定位、控制输出正常后，遥控解锁并切入OFFBOARD。
流程：上升0.5m稳定2秒 → 规划飞往(3,3,0.5) → 悬停2秒 → AUTO.LAND。
录包另开终端：bash ~/match_ws/shfiles/record_rrt_flight.sh
落地解除武装后Ctrl+C退出；空中异常请遥控接管，不要关闭本终端。
HELP
# 保持终端运行，由操作者在落地后结束本轮链路。
wait
