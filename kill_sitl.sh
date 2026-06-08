#!/usr/bin/env bash

echo "Stopping tmux SITL session..."
tmux kill-session -t px4_sitl_mission 2>/dev/null || true

echo "Stopping PX4, Gazebo, DDS agent, and mission executor..."
pkill -INT -f 'mission_executor_dds|MicroXRCEAgent|px4|PX4-Autopilot|gz sim|gzserver|gzclient|ign gazebo|ignition gazebo|ruby.*gz|ruby.*ign|gz sim server|gz sim gui' 2>/dev/null || true
sleep 2
pkill -TERM -f 'mission_executor_dds|MicroXRCEAgent|px4|PX4-Autopilot|gz sim|gzserver|gzclient|ign gazebo|ignition gazebo|ruby.*gz|ruby.*ign|gz sim server|gz sim gui' 2>/dev/null || true
sleep 2
pkill -KILL -f 'mission_executor_dds|MicroXRCEAgent|px4|PX4-Autopilot|gz sim|gzserver|gzclient|ign gazebo|ignition gazebo|ruby.*gz|ruby.*ign|gz sim server|gz sim gui' 2>/dev/null || true

echo "Stopping remaining Gazebo Harmonic processes..."
pkill -INT -f '^gz sim' 2>/dev/null || true
sleep 1
pkill -TERM -f '^gz sim' 2>/dev/null || true
sleep 1
pkill -KILL -f '^gz sim' 2>/dev/null || true

echo
echo "Remaining related processes:"
ps aux | grep -Ei 'mission_executor_dds|MicroXRCEAgent|px4|PX4-Autopilot|gz|gazebo|ignition|ruby.*gz|ruby.*ign' | grep -v grep || echo "None"
echo
echo "SITL cleanup done."
