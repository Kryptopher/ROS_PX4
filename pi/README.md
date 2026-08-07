# Raspberry Pi Profile

This folder contains Pi-specific wrappers and config examples. The ROS 2 package,
mission TSVs, mission-control UI, flight logger, and log tools are shared from
`../shared`.

The live ROS package should be linked into the ROS workspace from:

```bash
~/ROS_PX4/shared/zed_px4_bridge_folder
```

## Install The Shared ROS Package

```bash
mkdir -p ~/ros2_ws/src
ln -sfn ~/ROS_PX4/shared/zed_px4_bridge_folder ~/ros2_ws/src/zed_px4_bridge
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select zed_px4_bridge --symlink-install
source install/setup.bash
```

If `colcon` reports duplicate `zed_px4_bridge` packages, move the old copied
package outside `~/ros2_ws`, then rebuild. Do not copy
`shared/zed_px4_bridge_folder` into `pi/`.

## Configure The DDS Agent

Install `pi/config/dds-agent.service` as the user service for the Pi. This file
uses `/dev/ttyAMA0` on this Raspberry Pi because that serial device is present
on the machine. Confirm the PX4 DDS serial link before enabling the service:

```bash
ls -l /dev/serial0 /dev/ttyAMA0 /dev/ttyUSB0 /dev/ttyACM0 2>/dev/null
```

If PX4 is connected to another UART or USB serial adapter, edit
`pi/config/dds-agent.service` and change the `--dev` argument.

## Run A Real Hardware Mission

The Pi wrapper sets:

```bash
ROS_PX4_PLATFORM=pi
ROS_PX4_PLATFORM_HOME=~/ROS_PX4/pi
ROS_PX4_SHARED_HOME=~/ROS_PX4/shared
```

It also defaults `START_ENCODER=false`, so encoder hardware is not started
unless you explicitly enable it.

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

With explicit mission and safety limits:

```bash
cd ~/ROS_PX4/pi
MISSION_FILE=$HOME/ROS_PX4/shared/missions/Default.tsv \
START_ENCODER=false \
MISSION_DIR=$HOME/ROS_PX4/shared/missions \
SAFETY_MAX_ALTITUDE_M=50.0 \
SAFETY_WARN_ALTITUDE_M=48.0 \
SAFETY_MAX_VELOCITY_MS=7.0 \
SAFETY_LOCAL_RADIUS_M=50.0 \
scripts/run_dds_mission.sh "$MISSION_FILE"
```

Real hardware flight logs are written under `~/logs`.

## Run SITL

Copy `pi/config/sitl.env.example` to `~/.config/ros_px4/sitl.env` and set the
remote laptop/PX4 values for your network. Then run:

```bash
cd ~/ROS_PX4/pi
scripts/run_sitl
```

SITL mission logs are written under `~/.ros/mission_logs`.

## Platform Notes

Mission files live in `~/ROS_PX4/shared/missions`. Jetson UART repair and
hotspot tooling lives under `jetson/` and is not used by the Pi wrappers.
