#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
MODE="${MODE:-legged}"   # legged (full_system + MPC + 12-DoF gait) | walk (SRB torque MPC + 1kHz servo)
WORLD="${WORLD:-unity_plant}"   # unity_plant = Unity 발전소 배치(원점 = Unity Spot 시작점) | corridor | factory
if [ "$WORLD" = "unity_plant" ]; then
  DASH_WORLD=unity
else
  DASH_WORLD="$WORLD"
  # Unity 미러는 /odom을 절대 좌표로 Unity 시작점에 놓는다 — corridor(-20,0)·factory(-10,0) 스폰은 Unity 맵 밖이 된다
  echo "경고: Unity 미러(Plant_GazeboMirror)는 WORLD=unity_plant에서만 맵이 맞습니다. WORLD=$WORLD는 Gazebo·대시보드 확인용." >&2
fi
SLOPE_DEG="${SLOPE_DEG:-0.0}"
RTF="${RTF:-0.5}"

export PATH="/opt/homebrew/Caskroom/miniforge/base/envs/ros_env/bin:$PATH"
# macOS: prevent Gazebo Transport & FastDDS multicast failures
export GZ_IP=127.0.0.1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# shellcheck disable=SC1091
source /opt/homebrew/Caskroom/miniforge/base/etc/profile.d/conda.sh
conda activate ros_env
# shellcheck disable=SC1091
source "$REPO_ROOT/install/setup.bash"

if [ "$MODE" = "walk" ]; then
  echo "=== Starting Gazebo Headless + 12-DoF SRB Torque MPC (walk.launch.py world:=$WORLD slope_deg:=$SLOPE_DEG rtf:=$RTF) ==="
  ros2 launch mpc_controller walk.launch.py gui:=false "world:=$WORLD" "slope_deg:=$SLOPE_DEG" "real_time_factor:=$RTF" &
else
  echo "=== Starting Gazebo Headless + Full System 12-DoF Legged + MPC (full_system.launch.py locomotion:=legged world:=$WORLD) ==="
  ros2 launch simulation full_system.launch.py gui:=false locomotion:=legged inspection:=false gas_safety:=false "world:=$WORLD" &
fi
GZ_PID=$!

echo "=== Starting Native Gazebo <-> Unity <-> Three.js Bridge (:9090 WS, :8090 MJPEG) ==="
python3 "$REPO_ROOT/unity/tools/gazebo_unity_bridge.py" --host 127.0.0.1 --ws-port 9090 --video-port 8090 &
BRIDGE_PID=$!

echo "=== Starting Three.js Monitoring Dashboard (:8080) ==="
(cd "$REPO_ROOT/monitoring/web" && exec python3 -m http.server 8080 --bind 127.0.0.1) &
WEB_PID=$!

cleanup() {
  echo "Stopping Gazebo Mirror stack..."
  kill "$WEB_PID" "$BRIDGE_PID" "$GZ_PID" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  # ros2 launch가 SIGTERM에 legged 노드(elevation_map·gait·mpc·fall_recovery·patrol)를 남겨 고아가 된다
  pkill -f "walk_node|joint_servo|parameter_bridge|_node --ros-args|robot_state_publisher" 2>/dev/null || true
  wait "$WEB_PID" "$BRIDGE_PID" "$GZ_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo ""
echo "Ready!"
echo "  1) Unity: Open Assets/Scenes/Plant_GazeboMirror.unity and press Play"
echo "  2) Three.js: http://127.0.0.1:8080/?world=$DASH_WORLD"
echo "  3) Walk command example:"
echo "     ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.35}, angular: {z: 0.0}}'"
echo ""
wait
