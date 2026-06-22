#!/usr/bin/env bash
set -euo pipefail

REPO="${ROS_PX4_HOME:-$HOME/ROS_PX4}"
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"
MISSION_FILE="${MISSION_FILE:-${1:-$REPO/missions/hover_1m_test.tsv}}"
RUN_LABEL="${RUN_LABEL:-$(basename "$MISSION_FILE" .tsv)}"
SESSION="${DDS_SESSION:-px4_dds_mission}"
START_ENCODER="${START_ENCODER:-true}"
SAFETY_MAX_ALTITUDE_M="${SAFETY_MAX_ALTITUDE_M:-2.0}"
SAFETY_WARN_ALTITUDE_M="${SAFETY_WARN_ALTITUDE_M:-1.5}"
SAFETY_MAX_VELOCITY_MS="${SAFETY_MAX_VELOCITY_MS:-1.5}"
SAFETY_LOCAL_RADIUS_M="${SAFETY_LOCAL_RADIUS_M:-5.0}"

if [[ ! -f "$MISSION_FILE" ]]; then
  echo "Mission file not found: $MISSION_FILE" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"

if ! systemctl --user is-active --quiet dds-agent.service; then
  systemctl --user start dds-agent.service
  sleep 2
fi

if ! systemctl --user is-active --quiet dds-agent.service; then
  echo "The ARK TELEM2 DDS agent is not running. Check: systemctl --user status dds-agent.service" >&2
  exit 1
fi

for required_topic in /fmu/out/vehicle_status /fmu/out/vehicle_local_position; do
  if ! timeout 10 bash -c "until ros2 topic list | grep -qx '$required_topic'; do sleep 1; done"; then
    echo "No DDS data on $required_topic from the Pixhawk." >&2
    echo "Verify PX4 uXRCE-DDS is configured for TELEM2 at 3000000 baud and that TELEM2 is not also assigned to MAVLink." >&2
    exit 1
  fi
done

if ! ros2 pkg prefix px4_msgs >/dev/null 2>&1; then
  echo "px4_msgs is missing. Install/build it before starting a DDS mission." >&2
  exit 1
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n mission

tmux send-keys -t "$SESSION":0.0 "
source /opt/ros/humble/setup.bash
source '$ROS_WS/install/setup.bash'
ros2 run zed_px4_bridge mission_executor_dds --ros-args \
  -p mission_file:='$MISSION_FILE' \
  -p auto_arm:=false \
  -p auto_offboard:=false \
  -p auto_land:=false
" C-m

tmux split-window -h -t "$SESSION":0.0
tmux send-keys -t "$SESSION":0.1 "
source /opt/ros/humble/setup.bash
source '$ROS_WS/install/setup.bash'
ros2 run zed_px4_bridge safety_monitor --ros-args \
  -p max_altitude_m:=$SAFETY_MAX_ALTITUDE_M \
  -p warn_altitude_m:=$SAFETY_WARN_ALTITUDE_M \
  -p max_velocity_ms:=$SAFETY_MAX_VELOCITY_MS \
  -p local_radius_m:=$SAFETY_LOCAL_RADIUS_M
" C-m

tmux split-window -v -t "$SESSION":0.0
tmux send-keys -t "$SESSION":0.2 "
source /opt/ros/humble/setup.bash
source '$ROS_WS/install/setup.bash'
ros2 run zed_px4_bridge flight_logger --ros-args \
  -p mission_file:='$MISSION_FILE' \
  -p run_label:='$RUN_LABEL'
" C-m

tmux split-window -v -t "$SESSION":0.1
if [[ "$START_ENCODER" == "true" ]]; then
  tmux send-keys -t "$SESSION":0.3 "
source /opt/ros/humble/setup.bash
source '$ROS_WS/install/setup.bash'
ros2 run zed_px4_bridge payload_encoder
" C-m
else
  tmux send-keys -t "$SESSION":0.3 "echo 'Payload encoder disabled'; bash" C-m
fi

tmux select-layout -t "$SESSION":0 tiled
tmux attach-session -t "$SESSION"
