#!/usr/bin/env bash

# 原地起飞精降：一次启动基础链路和待命任务；start 子命令才触发实飞。
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SCRIPT_PATH="${SCRIPT_DIR}/start_uav1_inplace_landing_test.sh"

source_ros() {
  source /opt/ros/noetic/setup.bash
  source "${MATCH_WS}/devel/setup.bash"
  export ROS_MASTER_URI="http://${UAV1_SINGLE_ROS_MASTER_IP:-10.224.119.232}:11311"
  export ROS_IP="${UAV1_SINGLE_ROS_IP:-10.224.119.232}"
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
          allow_arming:=true auto_start:=false takeoff_height_m:=1.70 \
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
    source_ros
    echo '触发实飞：OFFBOARD → 解锁 → 原地起飞 1.70 米 → 精降。'
    exec rosservice call /landing_test/start '{}'
    ;;
  -h|--help)
    echo "全程NMPC测试：bash ${SCRIPT_PATH} --nmpc"
    echo "启动待命及画面：bash ${SCRIPT_PATH}"
    echo "定位正常、飞机放稳且遥控可接管后开始实飞：bash ${SCRIPT_PATH} start"
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
echo "准备好后在另一个终端执行：bash ${SCRIPT_PATH} start"
terminator --no-dbus --maximise --config="$runtime_config" --layout=uav1_inplace_landing_test
