#!/usr/bin/env bash
set -euo pipefail

REPO="${ROS_PX4_HOME:-$HOME/ROS_PX4}"
MISSION_FILE="${MISSION_FILE:-${1:-$REPO/missions/mission_sitl_test.tsv}}"
RUN_LABEL="${RUN_LABEL:-$(basename "$MISSION_FILE" .tsv)}"
SESSION="${DDS_SESSION:-px4_dds_mission}"
START_ENCODER="${START_ENCODER:-true}"

if [[ ! -f "$MISSION_FILE" ]]; then
  echo "Mission file not found: $MISSION_FILE" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

if ! ros2 pkg prefix px4_msgs >/dev/null 2>&1; then
  echo "px4_msgs is missing. Install/build it before starting a DDS mission." >&2
  exit 1
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n mission

tmux send-keys -t "$SESSION":0.0 "
source /opt/ros/humble/setup.bash
source '$REPO/install/setup.bash'
ros2 run zed_px4_bridge mission_executor_dds --ros-args \
  -p mission_file:='$MISSION_FILE' \
  -p auto_arm:=false \
  -p auto_offboard:=false \
  -p auto_land:=false
" C-m

tmux split-window -h -t "$SESSION":0.0
tmux send-keys -t "$SESSION":0.1 "
source /opt/ros/humble/setup.bash
source '$REPO/install/setup.bash'
ros2 run zed_px4_bridge safety_monitor
" C-m

tmux split-window -v -t "$SESSION":0.0
tmux send-keys -t "$SESSION":0.2 "
source /opt/ros/humble/setup.bash
source '$REPO/install/setup.bash'
ros2 run zed_px4_bridge flight_logger --ros-args \
  -p mission_file:='$MISSION_FILE' \
  -p run_label:='$RUN_LABEL'
" C-m

tmux split-window -v -t "$SESSION":0.1
if [[ "$START_ENCODER" == "true" ]]; then
  tmux send-keys -t "$SESSION":0.3 "
source /opt/ros/humble/setup.bash
source '$REPO/install/setup.bash'
ros2 run zed_px4_bridge payload_encoder
" C-m
else
  tmux send-keys -t "$SESSION":0.3 "echo 'Payload encoder disabled'; bash" C-m
fi

tmux select-layout -t "$SESSION":0 tiled
tmux attach-session -t "$SESSION"
