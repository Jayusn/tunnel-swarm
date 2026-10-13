#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/../../../devel/setup.bash"
# Dedicated local ROS graph, isolated from the normal flight master.
export ROS_MASTER_URI=http://127.0.0.1:11440
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
case "${1:-}" in
  sample)
    shift
    exec rosrun precision_landing sample_center_offset.py "$@"
    ;;
  *)
    exec roslaunch -p 11440 precision_landing center_offset_test.launch "$@"
    ;;
esac
