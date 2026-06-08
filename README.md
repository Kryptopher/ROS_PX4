# PX4 ROS2 DDS SITL Mission Executor

This repo contains a ROS2 Humble PX4 Offboard mission executor, a mission logbook monitor, SITL startup scripts, and example TSV mission files.

## Features

- Publishes PX4 Offboard heartbeat to `/fmu/in/offboard_control_mode`
- Publishes trajectory setpoints to `/fmu/in/trajectory_setpoint`
- Waits for PX4 to be both armed and in Offboard before starting the mission timer
- Keeps publishing the final setpoint after the mission reaches `end`
- Publishes mission lifecycle events on `/mission_executor/event`
- Logs PX4 state changes and mission executor events to CSV files

## Repository layout

```text
zed_px4_bridge/
  package.xml
  setup.py
  setup.cfg
  resource/zed_px4_bridge
  zed_px4_bridge/
    __init__.py
    mission_executor_dds.py
tools/
  mission_logbook.py
scripts/
  start_px4_sitl_mission.sh
  run_sitl
  kill_sitl.sh
missions/
  mission_sitl_test.tsv
  mission_sitl_done_test.tsv
```

## Install into an existing ROS2 workspace

Copy the package into your workspace:

```bash
mkdir -p ~/ros2_ws/src
cp -r zed_px4_bridge ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select zed_px4_bridge --symlink-install
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Copy scripts and mission files:

```bash
cp scripts/start_px4_sitl_mission.sh ~/start_px4_sitl_mission.sh
cp scripts/run_sitl ~/.local/bin/run_sitl
cp scripts/kill_sitl.sh ~/kill_sitl.sh
cp missions/mission_sitl_test.tsv ~/mission_sitl_test.tsv
cp missions/mission_sitl_done_test.tsv ~/mission_sitl_done_test.tsv

chmod +x ~/start_px4_sitl_mission.sh ~/.local/bin/run_sitl ~/kill_sitl.sh
```

Make sure `~/.local/bin` is in your PATH:

```bash
export PATH="$HOME/.local/bin:$PATH"
```

## Start SITL

```bash
run_sitl
```

In the PX4 `pxh>` shell, restart the QGC MAVLink link if needed:

```bash
mavlink stop -u 18570
mavlink start -x -u 18570 -r 4000000 -t 10.255.255.254 -o 14550 -f
mavlink boot_complete
```

Then start the mission:

```bash
commander mode offboard
commander arm
```

At mission end, the drone should hold the final setpoint. Land manually:

```bash
commander land
```

## Use a short test mission

```bash
MISSION_FILE=$HOME/mission_sitl_done_test.tsv run_sitl
```

## Run the logbook

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 tools/mission_logbook.py --label sitl_done_test --duration 60
```

Logs save to:

```text
~/.ros/mission_logs/
```

View latest events:

```bash
column -s, -t < "$(ls -t ~/.ros/mission_logs/*_events.csv | head -1)" | less -S
```

View latest samples:

```bash
column -s, -t < "$(ls -t ~/.ros/mission_logs/*_samples.csv | head -1)" | less -S
```

Exit the viewer with:

```text
q
```

## Mission file format

Mission files must be TAB-delimited `.tsv` files. The required columns are:

```text
t type mode profile x y z vx vy vz ax
```

Coordinate convention:

- Mission `x`: forward
- Mission `y`: right
- Mission `z`: positive up
- PX4 NED `z`: positive down, so the executor converts mission `z` to `-z`

Example:

```text
t	type	mode	profile	x	y	z	vx	vy	vz	ax
0	takeoff	pos	hold	0	0	1.0	0	0	0	0
5	wp	pos	hold	1	0	1.0	0	0	0	0
10	end	end	end	0	0	0	0	0	0	0
```

## Cleanup

```bash
~/kill_sitl.sh
```
