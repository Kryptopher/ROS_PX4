#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_DIR="${ROS_PX4_SHARED_HOME:-$(cd "$SCRIPT_DIR/.." && pwd)}"
ROOT_DIR="$(cd "$SHARED_DIR/.." && pwd)"
PLATFORM="${ROS_PX4_PLATFORM:-jetson}"
PLATFORM_DIR="${ROS_PX4_PLATFORM_HOME:-$ROOT_DIR/$PLATFORM}"
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"
MISSION_FILE="${MISSION_FILE:-${1:-$SHARED_DIR/missions/Default.tsv}}"
MISSION_DIR="${MISSION_DIR:-$SHARED_DIR/missions}"
RUN_LABEL="${RUN_LABEL:-real__$(basename "$MISSION_FILE" .tsv)}"
SESSION="${DDS_SESSION:-px4_dds_mission}"
START_ENCODER="${START_ENCODER:-true}"
START_MT6835_ENCODER="${START_MT6835_ENCODER:-false}"
MT6835_SERIAL_PORT="${MT6835_SERIAL_PORT:-/dev/ttyUSB0}"
MT6835_BAUD_RATE="${MT6835_BAUD_RATE:-115200}"
EXECUTOR_RATE_HZ="${EXECUTOR_RATE_HZ:-200.0}"
AUTO_LAND="${AUTO_LAND:-false}"
AUTO_SHUTDOWN="${AUTO_SHUTDOWN:-false}"
COMPLETION_SHUTDOWN_SCRIPT="$PLATFORM_DIR/scripts/stop_dds_mission.sh"
SAFETY_MAX_ALTITUDE_M="${SAFETY_MAX_ALTITUDE_M:-2.0}"
SAFETY_WARN_ALTITUDE_M="${SAFETY_WARN_ALTITUDE_M:-1.5}"
SAFETY_MAX_VELOCITY_MS="${SAFETY_MAX_VELOCITY_MS:-1.5}"
SAFETY_LOCAL_RADIUS_M="${SAFETY_LOCAL_RADIUS_M:-5.0}"
# ROS rejects an integer for a double parameter, which would crash safety_monitor.
for limit in SAFETY_MAX_ALTITUDE_M SAFETY_WARN_ALTITUDE_M SAFETY_MAX_VELOCITY_MS SAFETY_LOCAL_RADIUS_M; do
  if [[ "${!limit}" =~ ^[0-9]+$ ]]; then
    printf -v "$limit" '%s.0' "${!limit}"
  fi
done
MISSION_WEB_PORT="${MISSION_WEB_PORT:-8080}"
COPY_ULOG="${COPY_ULOG:-true}"
ULOG_SOURCE_DIR="${ULOG_SOURCE_DIR:-$HOME/.local/share/logloader/logs}"
ULOG_WAIT_S="${ULOG_WAIT_S:-10.0}"
FLIGHT_LOG_RATE_HZ="${FLIGHT_LOG_RATE_HZ:-100.0}"

if [[ ! -f "$MISSION_FILE" ]]; then
  echo "Mission file not found: $MISSION_FILE" >&2
  exit 1
fi

if [[ "$START_MT6835_ENCODER" == "true" ]]; then
  if [[ ! -c "$MT6835_SERIAL_PORT" ]]; then
    echo "MT6835 serial device is missing: $MT6835_SERIAL_PORT" >&2
    echo "Load the CH340/CH341 driver and reconnect the Nano:" >&2
    echo "  sudo modprobe usbserial" >&2
    echo "  sudo modprobe ch341 2>/dev/null || sudo insmod \"$HOME/ch341-driver/ch341.ko\"" >&2
    exit 1
  fi
  if [[ ! -r "$MT6835_SERIAL_PORT" || ! -w "$MT6835_SERIAL_PORT" ]]; then
    echo "MT6835 serial device is not readable and writable: $MT6835_SERIAL_PORT" >&2
    echo "Check that the current user belongs to the dialout group." >&2
    exit 1
  fi
fi

# ROS 2 setup scripts are not safe with Bash nounset enabled.
set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
export ROS_PX4_HOME="$SHARED_DIR"
set -u

if ! systemctl --user is-active --quiet dds-agent.service; then
  systemctl --user start dds-agent.service
  sleep 2
fi

if ! systemctl --user is-active --quiet dds-agent.service; then
  echo "The PX4 DDS agent is not running. Check: systemctl --user status dds-agent.service" >&2
  exit 1
fi

for required_topic_pattern in \
  '^/fmu/out/vehicle_status(_v[0-9]+)?$' \
  '^/fmu/out/vehicle_local_position(_v[0-9]+)?$'; do
  if ! timeout 10 bash -c "until ros2 topic list | grep -Eq '$required_topic_pattern'; do sleep 1; done"; then
    echo "No matching DDS topic for $required_topic_pattern from the Pixhawk." >&2
    echo "Verify PX4 uXRCE-DDS is configured for this platform and not assigned to another driver." >&2
    exit 1
  fi
done

if ! ros2 pkg prefix px4_msgs >/dev/null 2>&1; then
  echo "px4_msgs is missing. Install/build it before starting a DDS mission." >&2
  exit 1
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true

if [[ "$START_MT6835_ENCODER" == "true" ]] && command -v fuser >/dev/null 2>&1; then
  serial_users="$(fuser "$MT6835_SERIAL_PORT" 2>/dev/null || true)"
  if [[ -n "$serial_users" ]]; then
    echo "MT6835 serial device is already in use: $MT6835_SERIAL_PORT (PID(s):$serial_users)" >&2
    echo "Stop read_angles.py, Arduino Serial Monitor, or a standalone mt6835_encoder first." >&2
    exit 1
  fi
fi

tmux new-session -d -s "$SESSION" -n mission
EXECUTOR_PANE="$(tmux display-message -p -t "$SESSION":0.0 '#{pane_id}')"

tmux send-keys -t "$EXECUTOR_PANE" "
source /opt/ros/humble/setup.bash
source '$ROS_WS/install/setup.bash'
export ROS_PX4_HOME='$SHARED_DIR'
export DDS_SESSION='$SESSION'
ros2 run zed_px4_bridge mission_executor_dds --ros-args \
  -p mission_file:='$MISSION_FILE' \
  -p rate_hz:=$EXECUTOR_RATE_HZ \
  -p auto_arm:=false \
  -p auto_offboard:=false \
  -p auto_land:=$AUTO_LAND \
  -p auto_position_after_mission:=$AUTO_SHUTDOWN \
  -p completion_shutdown_script:=\"'$COMPLETION_SHUTDOWN_SCRIPT'\"
" C-m

SAFETY_PANE="$(tmux split-window -h -t "$EXECUTOR_PANE" -P -F '#{pane_id}')"
tmux send-keys -t "$SAFETY_PANE" "
source /opt/ros/humble/setup.bash
source '$ROS_WS/install/setup.bash'
export ROS_PX4_HOME='$SHARED_DIR'
ros2 run zed_px4_bridge safety_monitor --ros-args \
  -p max_altitude_m:=$SAFETY_MAX_ALTITUDE_M \
  -p warn_altitude_m:=$SAFETY_WARN_ALTITUDE_M \
  -p max_velocity_ms:=$SAFETY_MAX_VELOCITY_MS \
  -p local_radius_m:=$SAFETY_LOCAL_RADIUS_M
" C-m

LOGGER_PANE="$(tmux split-window -v -t "$EXECUTOR_PANE" -P -F '#{pane_id}')"
tmux send-keys -t "$LOGGER_PANE" "
source /opt/ros/humble/setup.bash
source '$ROS_WS/install/setup.bash'
export ROS_PX4_HOME='$SHARED_DIR'
ros2 run zed_px4_bridge flight_logger --ros-args \
  -p mission_file:='$MISSION_FILE' \
  -p run_label:='$RUN_LABEL' \
  -p sample_rate_hz:=$FLIGHT_LOG_RATE_HZ \
  -p copy_ulog:=$COPY_ULOG \
  -p ulog_source_dir:='$ULOG_SOURCE_DIR' \
  -p ulog_wait_s:=$ULOG_WAIT_S
" C-m

TERMINAL_UI_PANE="$(tmux split-window -v -t "$SAFETY_PANE" -P -F '#{pane_id}')"
tmux send-keys -t "$TERMINAL_UI_PANE" "
source /opt/ros/humble/setup.bash
source '$ROS_WS/install/setup.bash'
export ROS_PX4_HOME='$SHARED_DIR'
ros2 run zed_px4_bridge mission_control_ui --mission-dir '$MISSION_DIR' --shutdown-command '$PLATFORM_DIR/scripts/stop_dds_mission.sh'
" C-m

WEB_UI_PANE="$(tmux split-window -v -t "$SAFETY_PANE" -P -F '#{pane_id}')"
tmux send-keys -t "$WEB_UI_PANE" "
source /opt/ros/humble/setup.bash
source '$ROS_WS/install/setup.bash'
export ROS_PX4_HOME='$SHARED_DIR'
ros2 run zed_px4_bridge mission_control_web --mission-dir '$MISSION_DIR' --port '$MISSION_WEB_PORT' --shutdown-command '$PLATFORM_DIR/scripts/stop_dds_mission.sh'
" C-m

if [[ "$START_ENCODER" == "true" ]]; then
  ENCODER_PANE="$(tmux split-window -v -t "$SAFETY_PANE" -P -F '#{pane_id}')"
  tmux send-keys -t "$ENCODER_PANE" "
source /opt/ros/humble/setup.bash
source '$ROS_WS/install/setup.bash'
export ROS_PX4_HOME='$SHARED_DIR'
ros2 run zed_px4_bridge payload_encoder
" C-m
fi

if [[ "$START_MT6835_ENCODER" == "true" ]]; then
  MT6835_PANE="$(tmux split-window -v -t "$SAFETY_PANE" -P -F '#{pane_id}')"
  tmux send-keys -t "$MT6835_PANE" "
source /opt/ros/humble/setup.bash
source '$ROS_WS/install/setup.bash'
export ROS_PX4_HOME='$SHARED_DIR'
ros2 run zed_px4_bridge mt6835_encoder --ros-args \
  -p port:='$MT6835_SERIAL_PORT' \
  -p baud_rate:=$MT6835_BAUD_RATE
" C-m

  echo "Waiting for a live MT6835 encoder sample on /mt6835/angles..."
  if ! timeout 15 ros2 topic echo --no-daemon \
      /mt6835/angles std_msgs/msg/Float64MultiArray \
      --qos-profile sensor_data --once >/dev/null 2>&1; then
    echo "No MT6835 encoder sample received; refusing to continue." >&2
    echo "Encoder pane output:" >&2
    tmux capture-pane -p -t "$MT6835_PANE" -S -40 >&2 || true
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    exit 1
  fi
  echo "MT6835 encoder stream verified. Angles will be recorded while PX4 is in Offboard."
fi

echo "Checking that the critical mission nodes are running..."
for node_name in mission_executor_dds safety_monitor flight_logger; do
  if ! timeout 20 bash -c "until ros2 node list 2>/dev/null | grep -q '^/$node_name\$'; do sleep 1; done"; then
    echo "$node_name did not start; refusing to continue." >&2
    tmux capture-pane -p -J -t "$SESSION":0 -S -40 >&2 || true
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    exit 1
  fi
done
sleep 2
for node_name in mission_executor_dds safety_monitor flight_logger; do
  if ! pgrep -f "/zed_px4_bridge/$node_name( |\$)" >/dev/null; then
    echo "$node_name exited during startup; refusing to continue." >&2
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    exit 1
  fi
done

tmux select-layout -t "$SESSION":0 tiled
tmux attach-session -t "$SESSION"
