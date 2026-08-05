#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LDOP_ENV="/home/oem/ldop_ws/ldop_env.sh"
LOG_ROOT="${LDOP_LOG_ROOT:-/home/oem/ldop_logs}"
PID_FILE="${LOG_ROOT}/.flight_logger.pid"

if [[ ! -f "${LDOP_ENV}" ]]; then
    echo "找不到 LDOP 环境脚本：${LDOP_ENV}" >&2
    exit 1
fi
source "${LDOP_ENV}"
set -u

mkdir -p "${LOG_ROOT}"

if [[ -f "${PID_FILE}" ]]; then
    OLD_PID="$(cat "${PID_FILE}" 2>/dev/null || true)"
    if [[ "${OLD_PID}" =~ ^[0-9]+$ ]] && kill -0 "${OLD_PID}" 2>/dev/null; then
        echo "日志记录器已经在运行，PID=${OLD_PID}"
        exit 0
    fi
    rm -f "${PID_FILE}"
fi

STAMP="$(date +%Y%m%d_%H%M%S_%N)"
LOG_DIR="${LOG_ROOT}/${STAMP}"
mkdir -p "${LOG_DIR}"

echo "log_dir=${LOG_DIR}" > "${LOG_ROOT}/current_flight.txt"
python3 -u "${SCRIPT_DIR}/ros_flight_logger.py" --log-dir "${LOG_DIR}" \
    > "${LOG_DIR}/logger_console.log" 2>&1 &
LOGGER_PID=$!
echo "${LOGGER_PID}" > "${PID_FILE}"
echo "pid=${LOGGER_PID}" >> "${LOG_ROOT}/current_flight.txt"

cleanup() {
    rm -f "${PID_FILE}"
}
trap cleanup EXIT

echo "LDOP 飞行日志已启动，目录：${LOG_DIR}，PID=${LOGGER_PID}"
wait "${LOGGER_PID}"
