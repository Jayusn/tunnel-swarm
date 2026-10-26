#!/usr/bin/env bash

# 固定双机 ROS 网络后，调用当前 UAV0 七分屏入口。

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NETWORK_SETUP="${SCRIPT_DIR}/configure_dual_uav_ros_network.sh"
UAV0_LAUNCHER="${SCRIPT_DIR}/start_uav0_six_terminator.sh"

[[ -r "${NETWORK_SETUP}" ]] || { printf '找不到 %s\n' "${NETWORK_SETUP}" >&2; exit 1; }
[[ -x "${UAV0_LAUNCHER}" ]] || { printf '找不到 %s\n' "${UAV0_LAUNCHER}" >&2; exit 1; }

# shellcheck disable=SC1090
source "${NETWORK_SETUP}"
configure_dual_uav_ros_network uav0

# 双机检测栈会启动降落控制仲裁器。正常阶段由它透传 AutoTrans 姿态指令；
# 精降触发后停止透传姿态指令，只向 PX4 转发精降 PositionTarget。
export UAV0_AUTOTRANS_SETPOINT_TOPIC="/UAV0/control/attitude_setpoint"

# 正式总流程直接启用确认出口后的锁存、穿越和搜索降落，覆盖旧观察模式环境变量。
export UAV0_ENABLE_AUTO_EXIT=true

# 前机用当前地图核验实飞路线上的停留空间，持续发布普通/障碍后接力点。
export UAV0_SAFE_RELAY_ENABLED=true

# 第五屏只保留降落板搜索链路：D435 前视粗搜、下视相机搜索/确认和精降。
# 颜色标签、二维码、热成像及其目标上报均不在双机六分屏任务中启动。
export UAV0_FIRST_SEVEN_ENABLE_COLOR_TAG=false
export UAV0_FIRST_SEVEN_ENABLE_QR=false
export UAV0_FIRST_SEVEN_THERMAL=false
export UAV0_FIRST_SEVEN_ENABLE_TARGET_REPORTING=false

# 默认只录轻量规划诊断；完整建图回放需显式设置 UAV0_PLANNER_ROSBAG_CLOUDS=true。
export UAV0_PLANNER_ROSBAG_PREFIX="${UAV0_PLANNER_ROSBAG_PREFIX:-${HOME}/.ros/corridor_replay/uav0_corridor}"
export UAV0_PLANNER_ROSBAG_EXTRA_TOPICS="${UAV0_PLANNER_ROSBAG_EXTRA_TOPICS:-/UAV1/fast_lio/Odom_high_freq /UAV0/move_base_simple/goal /UAV0/mavros/state /UAV0/mavros/rc/in /UAV0/mavros/local_position/odom /UAV1/mavros/state /UAV1/mavros/rc/in /mission/rc_land_active}"
if [[ "${UAV0_PLANNER_ROSBAG_CLOUDS:-false}" == true ]]; then
  export UAV0_PLANNER_ROSBAG_EXTRA_TOPICS="${UAV0_PLANNER_ROSBAG_EXTRA_TOPICS} /UAV0/fast_lio/cloud_registered /UAV1/fast_lio/cloud_registered"
fi
# 同时记录接力点快照、有效性状态和实飞参考路线；保留外部自定义的附加话题。
export UAV0_PLANNER_ROSBAG_EXTRA_TOPICS="${UAV0_PLANNER_ROSBAG_EXTRA_TOPICS} /UAV0/mission/relay_waypoints /UAV0/mission/relay_status /UAV0/mission/reference_route /UAV0/mission/relay_labels"

exec "${UAV0_LAUNCHER}" "$@"
