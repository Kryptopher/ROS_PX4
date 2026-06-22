#!/usr/bin/env bash
set -e

SESSION="px4_sitl_mission"
REPO="${ROS_PX4_HOME:-$HOME/ROS_PX4}"
CONFIG_FILE="${SITL_CONFIG_FILE:-$HOME/.config/ros_px4/sitl.env}"
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"

if [[ -f "$CONFIG_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
fi

MISSION_FILE="${MISSION_FILE:-$REPO/missions/mission_sitl_test.tsv}"
MISSION_NAME="$(basename "$MISSION_FILE" .tsv)"
RUN_NAME="${RUN_NAME:-${LOGBOOK_LABEL:-$MISSION_NAME}}"
LOGBOOK_LABEL="sitl__${RUN_NAME}"
LOGBOOK_RATE_HZ="${LOGBOOK_RATE_HZ:-100}"
EXECUTOR_RATE_HZ="${EXECUTOR_RATE_HZ:-50.0}"
SITL_MODE="${SITL_MODE:-remote}"
SITL_GUI="${SITL_GUI:-false}"
SITL_REMOTE_PX4_DIR="${SITL_REMOTE_PX4_DIR:-~/PX4-Autopilot}"
SITL_AGENT_IP="${SITL_AGENT_IP:-}"

px4_ipv4_param_value() {
  local ip="$1"
  local a b c d value
  IFS=. read -r a b c d <<<"$ip"
  if [[ ! "$a" =~ ^[0-9]+$ || ! "$b" =~ ^[0-9]+$ || ! "$c" =~ ^[0-9]+$ || ! "$d" =~ ^[0-9]+$ ]]; then
    echo "Invalid IPv4 address: $ip" >&2
    return 1
  fi
  value=$((a * 16777216 + b * 65536 + c * 256 + d))
  if ((value >= 2147483648)); then
    value=$((value - 4294967296))
  fi
  printf '%s\n' "$value"
}

if [[ "$SITL_MODE" == "remote" ]]; then
  if [[ -z "${SITL_HOST:-}" ]]; then
    echo "Remote SITL is the default, but SITL_HOST is not configured." >&2
    echo "Create $CONFIG_FILE from $REPO/config/sitl.env.example." >&2
    echo "Use SITL_MODE=local run_sitl to run SITL on the Pi." >&2
    exit 1
  fi

  if [[ -z "$SITL_AGENT_IP" ]]; then
    echo "SITL_AGENT_IP is not configured in $CONFIG_FILE." >&2
    echo "Set it to the Pi IP address reachable from the laptop." >&2
    exit 1
  fi

  if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$SITL_HOST" true; then
    echo "Cannot SSH into $SITL_HOST from the Pi." >&2
    echo "Enable the laptop/WSL SSH server and configure key-based access." >&2
    exit 1
  fi

  if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$SITL_HOST" \
      "test -d $SITL_REMOTE_PX4_DIR"; then
    echo "Cannot access PX4 directory '$SITL_REMOTE_PX4_DIR' on $SITL_HOST." >&2
    echo "Verify the remote PX4 path in $CONFIG_FILE." >&2
    exit 1
  fi

  # Clear stale remote SITL processes from a prior tmux/SSH disconnect. PX4
  # refuses to start when another instance server is still alive.
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$SITL_HOST" \
    "pkill -INT -x px4 2>/dev/null || true; \
     pkill -INT -f '^gz sim( |$)' 2>/dev/null || true; \
     pkill -INT -x gzserver 2>/dev/null || true; \
     pkill -INT -x gzclient 2>/dev/null || true" \
    2>/dev/null || true
  sleep 1
fi

if [[ "$SITL_GUI" == "true" ]]; then
  SITL_MAKE_COMMAND="make px4_sitl gz_x500"
else
  SITL_MAKE_COMMAND="HEADLESS=1 make px4_sitl gz_x500"
fi

if [[ "$SITL_MODE" == "remote" ]]; then
  SITL_AGENT_IP_PARAM="$(px4_ipv4_param_value "$SITL_AGENT_IP")"
  if [[ "$SITL_GUI" == "true" ]]; then
    # SSH sessions into WSL do not inherit the WSLg display environment.
    SITL_MAKE_COMMAND="DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir PULSE_SERVER=unix:/mnt/wslg/PulseServer $SITL_MAKE_COMMAND"
  fi
  SITL_MAKE_COMMAND="PX4_PARAM_UXRCE_DDS_AG_IP=$SITL_AGENT_IP_PARAM $SITL_MAKE_COMMAND"
  PX4_SITL_COMMAND="ssh -tt $SITL_HOST \"cd $SITL_REMOTE_PX4_DIR && $SITL_MAKE_COMMAND\""
else
  PX4_SITL_COMMAND="cd ~/PX4-Autopilot && $SITL_MAKE_COMMAND"
fi

# Gracefully stop an existing logbook so its buffered samples are saved.
pkill -INT -f "$REPO/tools/mission_logbook.py" 2>/dev/null || true
sleep 1
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n sitl
AGENT_PANE="$(tmux display-message -p -t "$SESSION":0.0 '#{pane_id}')"

# Pane 0: MicroXRCEAgent
tmux send-keys -t "$AGENT_PANE" '
MicroXRCEAgent udp4 -p 8888
' C-m

# Pane 1: PX4 SITL
PX4_PANE="$(tmux split-window -h -t "$AGENT_PANE" -P -F '#{pane_id}')"
tmux send-keys -t "$PX4_PANE" '
'"${PX4_SITL_COMMAND}"'
' C-m

# Allow SITL missions without QGroundControl and route remote PX4 DDS to the Pi.
(
  for _ in $(seq 1 180); do
    if tmux capture-pane -p -t "$PX4_PANE" -S -20 | grep -q 'pxh>'; then
      break
    fi
    sleep 1
  done

  # PX4's SITL startup script launches the uXRCE-DDS client with a loopback
  # address.  In remote SITL, reconnect it after startup so it reaches the
  # MicroXRCEAgent running on this Jetson.
  if [[ "$SITL_MODE" == "remote" ]]; then
    tmux send-keys -t "$PX4_PANE" "uxrce_dds_client stop" C-m
    tmux send-keys -t "$PX4_PANE" \
      "uxrce_dds_client start -t udp -h $SITL_AGENT_IP -p 8888" C-m
  fi

  tmux send-keys -t "$PX4_PANE" "param set NAV_DLL_ACT 0" C-m
) &

# Pane 2: Mission executor.
# It waits for PX4 local position, then waits until PX4 is armed and in Offboard
# before starting the mission timer.
EXECUTOR_PANE="$(tmux split-window -v -t "$PX4_PANE" -P -F '#{pane_id}')"
tmux send-keys -t "$EXECUTOR_PANE" "
sleep 12
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
ros2 run zed_px4_bridge mission_executor_dds --ros-args \\
  -p mission_file:=$MISSION_FILE \\
  -p rate_hz:=$EXECUTOR_RATE_HZ \\
  -p auto_arm:=false \\
  -p auto_offboard:=false \\
  -p auto_land:=true \\
  -p auto_disarm_after_land:=true \\
  -p force_disarm_after_land_timeout_s:=15.0 \\
  -p start_position_tolerance_m:=0.12 \\
  -p start_velocity_tolerance_ms:=0.15 \\
  -p start_settle_time_s:=1.5
" C-m

# Pane 3: Mission state logbook
LOGBOOK_PANE="$(tmux split-window -v -t "$AGENT_PANE" -P -F '#{pane_id}')"
tmux send-keys -t "$LOGBOOK_PANE" "
sleep 15
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
python3 '$REPO/tools/mission_logbook.py' --label '$LOGBOOK_LABEL' \\
  --sample-rate-hz '$LOGBOOK_RATE_HZ' \\
  --mission-file '$MISSION_FILE'
" C-m

tmux select-layout -t "$SESSION":0 tiled
tmux attach-session -t "$SESSION"
