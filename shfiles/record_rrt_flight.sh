#!/usr/bin/env bash
# 记录RRT实飞数据；不启动飞行节点或发送控制指令。
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../devel/setup.bash"
source "${SCRIPT_DIR}/configure_dual_uav_ros_network.sh"
configure_dual_uav_ros_network uav0
mkdir -p "${HOME}/flight_bags"
cd "${HOME}/flight_bags"
df -h .
printf '录包目录：%s\n每约1GB分包；落地解除武装后按Ctrl+C停止并保存。\n' "$PWD"
exec rosbag record --split --size=1024 \
  -o rrt_flight \
  /UAV0/fast_lio/cloud_registered \
  /UAV0/fast_lio/Odometry \
  /UAV0/fast_lio/Odom_high_freq \
  /UAV0/rrt/scene \
  /UAV0/rrt/mission_goal \
  /UAV0/rrt/mission_status \
  /UAV0/rrt/polynomial \
  /UAV0/rrt/path \
  /UAV0/planning/pos_cmd \
  /UAV0/control/position_setpoint \
  /UAV0/landing/control_owner \
  /UAV0/mavros/setpoint_raw/local \
  /UAV0/mavros/state \
  /UAV0/mavros/extended_state \
  /UAV0/mavros/local_position/odom \
  /UAV0/mavros/vision_pose/pose \
  /UAV0/mavros/rc/in \
  /UAV0/mavros/battery \
  /tf /tf_static /rosout_agg
