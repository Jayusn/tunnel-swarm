#!/usr/bin/env bash

# 18 cm、DICT_4X4_50、相对起飞点 2 m 的独立降落测试。
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export UAV1_TEST_ENTRY_SCRIPT="${SCRIPT_DIR}/start_uav1_18cm_landing_test.sh"
export UAV1_TEST_TAKEOFF_HEIGHT_M=2.0
export UAV1_TEST_MARKER_SIZE_M=0.18
export UAV1_TEST_MARKER_DICTIONARY=DICT_4X4_50
exec bash "${SCRIPT_DIR}/start_uav1_inplace_landing_test.sh" "$@"
