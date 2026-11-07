#!/usr/bin/env bash

# 18 cm、DICT_4X4_50、1.5 m；手动解锁后由遥控器 OFFBOARD 触发起飞。
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export UAV1_TEST_ENTRY_SCRIPT="${SCRIPT_DIR}/start_uav1_18cm_landing_test.sh"
export UAV1_TEST_TAKEOFF_HEIGHT_M=1.5
export UAV1_TEST_MARKER_SIZE_M=0.18
export UAV1_TEST_MARKER_DICTIONARY=DICT_4X4_50
export UAV1_TEST_START_ON_OFFBOARD=true
export UAV1_TEST_ALLOW_ARMING=false
exec bash "${SCRIPT_DIR}/start_uav1_inplace_landing_test.sh" "$@"
