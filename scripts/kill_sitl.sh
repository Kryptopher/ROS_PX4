#!/usr/bin/env bash

CONFIG_FILE="${SITL_CONFIG_FILE:-$HOME/.config/ros_px4/sitl.env}"
if [[ -f "$CONFIG_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$CONFIG_FILE"
fi

signal_simulation() {
    local signal="$1"
    pkill "-$signal" -f '/mission_executor_dds( |$)' 2>/dev/null || true
    pkill "-$signal" -x MicroXRCEAgent 2>/dev/null || true
    pkill "-$signal" -x px4 2>/dev/null || true
    pkill "-$signal" -x gz 2>/dev/null || true
    pkill "-$signal" -f '^gz sim( |$)' 2>/dev/null || true
    pkill "-$signal" -x gzserver 2>/dev/null || true
    pkill "-$signal" -x gzclient 2>/dev/null || true
}

signal_remote_simulation() {
    local signal="$1"
    if [[ "${SITL_MODE:-remote}" == "remote" && -n "${SITL_HOST:-}" ]]; then
        ssh -o BatchMode=yes -o ConnectTimeout=3 "$SITL_HOST" \
            "pkill -$signal -x px4 2>/dev/null || true; \
             pkill -$signal -f '^gz sim( |$)' 2>/dev/null || true; \
             pkill -$signal -x gzserver 2>/dev/null || true; \
             pkill -$signal -x gzclient 2>/dev/null || true" \
            2>/dev/null || true
    fi
}

echo "Saving mission logbook..."
pkill -INT -f 'tools/mission_logbook.py' 2>/dev/null || true
sleep 2

echo "Stopping PX4, Gazebo, DDS agent, and mission executor..."
signal_remote_simulation INT
signal_simulation INT
sleep 2
signal_remote_simulation TERM
signal_simulation TERM
sleep 2
signal_remote_simulation KILL
signal_simulation KILL

echo "Stopping tmux SITL session..."
tmux kill-session -t px4_sitl_mission 2>/dev/null || true

echo
echo "Remaining related processes:"
ps aux | grep -Ei 'mission_logbook|mission_executor_dds|MicroXRCEAgent|px4|PX4-Autopilot|gz|gazebo|ignition|ruby.*gz|ruby.*ign' | grep -vE 'grep|kill_sitl.sh' || echo "None"
echo
echo "SITL cleanup done."
