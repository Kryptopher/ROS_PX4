#!/usr/bin/env bash
set -e

SESSION="px4_sitl_mission"
MISSION_FILE="${MISSION_FILE:-$HOME/mission_sitl_test.tsv}"

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n sitl

# Pane 0: MicroXRCEAgent
tmux send-keys -t "$SESSION":0.0 '
MicroXRCEAgent udp4 -p 8888
' C-m

# Pane 1: PX4 SITL
tmux split-window -h -t "$SESSION":0.0
tmux send-keys -t "$SESSION":0.1 '
cd ~/PX4-Autopilot
make px4_sitl gz_x500
' C-m

# Pane 2: Mission executor.
# It waits for PX4 local position, then waits until PX4 is armed and in Offboard
# before starting the mission timer.
tmux split-window -v -t "$SESSION":0.1
tmux send-keys -t "$SESSION":0.2 "
sleep 12
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run zed_px4_bridge mission_executor_dds --ros-args \\
  -p mission_file:=$MISSION_FILE \\
  -p auto_arm:=false \\
  -p auto_offboard:=false
" C-m

# Pane 3: ROS check shell
tmux split-window -v -t "$SESSION":0.0
tmux send-keys -t "$SESSION":0.3 '
sleep 15
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
echo
echo "Useful checks:"
echo "  ros2 topic hz /fmu/in/offboard_control_mode"
echo "  ros2 topic hz /fmu/in/trajectory_setpoint"
echo "  ros2 topic echo /mission_executor/event"
echo
bash
' C-m

tmux select-layout -t "$SESSION":0 tiled
tmux attach-session -t "$SESSION"
