#!/usr/bin/env bash

# 保留原入口路径，本机恢复 UAV1 后机角色，使用 UAV1 双机七分屏。
# 默认连接前机 10.59.40.213，本机地址为 10.59.40.233。

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec bash "${SCRIPT_DIR}/start_uav1_six_terminator.sh" "$@"
