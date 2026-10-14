#!/usr/bin/env bash

set -o pipefail

landing_log_dir="${PRECISION_LANDING_LOG_DIR:-${HOME}/target_reports/landing_tests}"
mkdir -p "${landing_log_dir}" || exit 1

landing_timestamp="$(date +%Y%m%d_%H%M%S)"
landing_log_file="$(mktemp "${landing_log_dir}/landing_${landing_timestamp}_XXXXXX.log")" ||
  exit 1

echo "Landing log: ${landing_log_file}"
roslaunch uav1_inplace_landing_test landing_test.launch "$@" 2>&1 |
  tee "${landing_log_file}"

exit "${PIPESTATUS[0]}"
