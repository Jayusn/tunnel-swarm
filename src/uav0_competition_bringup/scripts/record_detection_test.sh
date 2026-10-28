#!/usr/bin/env bash
# roslaunch manages the recorder PID directly after exec, including Ctrl+C.
set -e
bag_prefix=$1
shift
mkdir -p -- "$(dirname -- "$bag_prefix")"
exec rosbag record --lz4 --split --size=1024 --buffsize=512 -O "$bag_prefix" "$@"
