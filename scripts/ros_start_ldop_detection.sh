#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LIDAR_SCRIPT="${SCRIPT_DIR}/ros_start_lidar_only.sh"
LDOP_ENV="/home/oem/ldop_ws/ldop_env.sh"

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "错误：找不到 gnome-terminal。" >&2
    exit 1
fi
if ! command -v timeout >/dev/null 2>&1; then
    echo "错误：找不到 timeout，无法执行带超时的话题等待。" >&2
    exit 1
fi
if [[ ! -x "${LIDAR_SCRIPT}" ]]; then
    echo "错误：雷达启动脚本不存在或不可执行：${LIDAR_SCRIPT}" >&2
    exit 1
fi
if [[ ! -f "${LDOP_ENV}" ]]; then
    echo "错误：LDOP 环境脚本不存在：${LDOP_ENV}" >&2
    exit 1
fi

"${LIDAR_SCRIPT}"

gnome-terminal --tab --title="5_LDOP" -- bash -c '
    # ROS 的环境脚本可能引用尚未定义的变量，先不要启用 nounset。
    set -o pipefail
    trap "exit 130" INT TERM
    if ! source "/home/oem/ldop_ws/ldop_env.sh"; then
        echo "错误：加载 LDOP 环境失败。"
        exec bash
    fi
    set -u

    wait_for_topic() {
        local topic="$1"
        echo "等待 ${topic} 发布实际消息..."
        until timeout 3s rostopic echo -n 1 "${topic}" >/dev/null 2>&1; do
            sleep 1
        done
        echo "${topic} 已有实际消息。"
    }

    wait_for_topic /cloud_registered
    wait_for_topic /Odometry
    echo "========== 5_LDOP =========="
    echo "roslaunch ldop run_ldop.launch"
    roslaunch ldop run_ldop.launch
    exec bash
'
