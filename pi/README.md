# Raspberry Pi Profile

This folder contains Pi-specific wrappers and config examples. The ROS 2 package,
mission TSVs, and log tools are shared from `../shared`.

## Install The Shared ROS Package

```bash
mkdir -p ~/ros2_ws/src
ln -sfn ~/ROS_PX4/shared/zed_px4_bridge_folder ~/ros2_ws/src/zed_px4_bridge
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select zed_px4_bridge --symlink-install
```

## Run A Mission

```bash
cd ~/ROS_PX4/pi
START_ENCODER=false scripts/run_dds_mission.sh
```

The Pi wrapper uses `../shared/missions` by default. Override the selected
mission the same way as Jetson:

```bash
MISSION_FILE=$HOME/ROS_PX4/shared/missions/square.tsv \
scripts/run_dds_mission.sh
```

## Platform Notes

Update `pi/config/dds-agent.service` for the Pi serial device connected to PX4.
Common Pi UART names are `/dev/ttyAMA0` and `/dev/serial0`; confirm the correct
port before enabling the service.
