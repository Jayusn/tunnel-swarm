#!/usr/bin/env bash

# 原地起飞精降：默认由 start 触发，18 cm 入口由手动解锁及 OFFBOARD 触发。
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SCRIPT_PATH="${UAV1_TEST_ENTRY_SCRIPT:-${SCRIPT_DIR}/start_uav1_inplace_landing_test.sh}"
TAKEOFF_HEIGHT_M="${UAV1_TEST_TAKEOFF_HEIGHT_M:-1.50}"
START_ON_OFFBOARD="${UAV1_TEST_START_ON_OFFBOARD:-false}"
# 所有分屏共用本机地址，基础四屏也通过环境变量继承，避免各用旧默认值。
export UAV1_SINGLE_ROS_IP="${UAV1_SINGLE_ROS_IP:-192.168.230.232}"
export UAV1_SINGLE_ROS_MASTER_IP="${UAV1_SINGLE_ROS_MASTER_IP:-${UAV1_SINGLE_ROS_IP}}"

show_start_instruction() {
  if [[ "$START_ON_OFFBOARD" == true ]]; then
    echo "定位和画面正常后，先手动解锁，再用遥控器切入 OFFBOARD：起飞 ${TAKEOFF_HEIGHT_M} 米后精降。"
  else
    echo "准备好后在另一个终端执行：bash ${SCRIPT_PATH} start"
  fi
}

source_ros() {
  source /opt/ros/noetic/setup.bash
  source "${MATCH_WS}/devel/setup.bash"
  export ROS_MASTER_URI="http://${UAV1_SINGLE_ROS_MASTER_IP}:11311"
  export ROS_IP="${UAV1_SINGLE_ROS_IP}"
  unset ROS_HOSTNAME
}

case "${1:-}" in
  --pane)
    case "${2:-}" in
      mavros|mid360|fastlio|pose)
        exec bash "${SCRIPT_DIR}/start_uav1_seven_terminator.sh" --pane "$2"
        ;;
      landing)
        source_ros
        # MAVROS 所在分屏负责启动 master，避免多个 roslaunch 同时创建它。
        until rosnode list >/dev/null 2>&1; do sleep 1; done
        exec rosrun uav1_inplace_landing_test start_landing_test.sh \
          allow_arming:="${UAV1_TEST_ALLOW_ARMING:-true}" auto_start:=false \
          start_on_offboard:="${START_ON_OFFBOARD}" takeoff_height_m:="${TAKEOFF_HEIGHT_M}" \
          marker_size_m:="${UAV1_TEST_MARKER_SIZE_M:-0.50}" \
          marker_dictionary:="${UAV1_TEST_MARKER_DICTIONARY:-DICT_4X4_250}" \
          enable_nmpc_descent:="${UAV1_TEST_NMPC_DESCENT:-true}"
        ;;
      image)
        source_ros
        until rosnode list >/dev/null 2>&1; do sleep 1; done
        sh "${SCRIPT_DIR}/run_uav1_sensor_stack.sh" landing &
        camera_pid=$!
        trap 'kill -INT "$camera_pid" 2>/dev/null || true' EXIT INT TERM
        rosrun rqt_image_view rqt_image_view /UAV1/landing/xyz_image
        exit $?
        ;;
      *) echo '未知分屏' >&2; exit 2 ;;
    esac
    ;;
  --nmpc) export UAV1_TEST_NMPC_DESCENT=true ;;
  start)
    if [[ "$START_ON_OFFBOARD" == true ]]; then
      show_start_instruction
      exit 2
    fi
    source_ros
    echo "触发实飞：OFFBOARD → 解锁 → 原地起飞 ${TAKEOFF_HEIGHT_M} 米 → 精降。"
    exec rosservice call /landing_test/start '{}'
    ;;
  -h|--help)
    echo "全程NMPC测试：bash ${SCRIPT_PATH} --nmpc"
    echo "启动待命及画面：bash ${SCRIPT_PATH}"
    show_start_instruction
    exit 0
    ;;
  '') ;;
  *) echo '未知参数，请使用 --help' >&2; exit 2 ;;
esac

command -v terminator >/dev/null
[[ -n "${DISPLAY:-}" ]] || { echo '请在图形桌面终端运行' >&2; exit 1; }
runtime_config="$(mktemp /tmp/uav1_inplace_landing_XXXXXX.conf)"
trap 'rm -f -- "$runtime_config"' EXIT
sed "s|__UAV1_INPLACE_TEST_SCRIPT__|bash ${SCRIPT_PATH}|g" \
  "${SCRIPT_DIR}/terminator_uav1_inplace_landing_test.conf" > "$runtime_config"
echo '启动 MAVROS、MID360、FAST-LIO、位姿回传、精降待命和下视画面。'
echo '请先关闭手持标定和其他飞行控制流程；勿重复启动。'
show_start_instruction
terminator --no-dbus --maximise --config="$runtime_config" --layout=uav1_inplace_landing_test
