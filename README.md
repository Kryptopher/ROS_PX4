# ROS_PX4

This repository is split into shared mission code plus platform-specific setup.

```text
shared/   Common ROS 2 package, mission TSVs, launch scripts, and log tools.
jetson/   Jetson-specific configs, hardware notes, and wrapper scripts.
pi/       Raspberry Pi-specific configs, notes, and wrapper scripts.
```

Install the ROS package from `shared/zed_px4_bridge_folder` on either platform:

```bash
mkdir -p ~/ros2_ws/src
ln -sfn ~/ROS_PX4/shared/zed_px4_bridge_folder ~/ros2_ws/src/zed_px4_bridge
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select zed_px4_bridge --symlink-install
```

Use the platform wrapper for the computer you are flying from:

```bash
cd ~/ROS_PX4/jetson
scripts/run_dds_mission.sh
```

or:

```bash
cd ~/ROS_PX4/pi
scripts/run_dds_mission.sh
```

Mission files live in `shared/missions`.
