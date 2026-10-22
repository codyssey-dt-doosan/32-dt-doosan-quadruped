#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/workspace/inspection/gauge_ocr:${PYTHONPATH:-}"
python3 /workspace/unity-tools/inspection_ros.py &
inspection_pid=$!
python3 /workspace/unity-tools/inspection_patrol.py &
patrol_pid=$!
cleanup() { kill "$inspection_pid" "$patrol_pid" 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT
# If either node exits, stop the other as well. Unity's command watchdog stops motion.
wait -n "$inspection_pid" "$patrol_pid"
