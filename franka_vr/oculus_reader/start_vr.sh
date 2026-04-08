#!/bin/bash

# 获取脚本所在的目录并切换到那里
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
cd "$SCRIPT_DIR" || exit 1

# 以脚本方式启动，并补齐包搜索路径
cd "$SCRIPT_DIR/oculus_reader" || exit 1
export PYTHONPATH="$SCRIPT_DIR:${PYTHONPATH:-}"
python3 start_franka_vr.py