# PX4 ROS2 DDS SITL Mission Executor

This repo contains a ROS2 Humble PX4 Offboard mission executor, a mission logbook monitor, SITL startup scripts, and example TSV mission files.

## Features

- Publishes PX4 Offboard heartbeat to `/fmu/in/offboard_control_mode`
- Publishes trajectory setpoints to `/fmu/in/trajectory_setpoint`
- Holds the first takeoff setpoint until PX4 is armed, in Offboard, and settled
  at the requested position before starting the mission timer
- Keeps publishing the final setpoint after the mission reaches `end`
- Publishes mission lifecycle events on `/mission_executor/event`
- Logs PX4 state changes and mission executor events to CSV files
- Runs an independent DDS safety watchdog with RTL abort
- Logs synchronized PX4 flight state and payload encoder angles
- Publishes dual quadrature payload encoder angles at 200 Hz

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
  run_dds_mission.sh
  start_px4_sitl_mission.sh
  run_sitl
  kill_sitl
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
ln -sf ~/ROS_PX4/scripts/kill_sitl.sh ~/.local/bin/kill_sitl
cp missions/mission_sitl_test.tsv ~/mission_sitl_test.tsv
cp missions/mission_sitl_done_test.tsv ~/mission_sitl_done_test.tsv

chmod +x ~/start_px4_sitl_mission.sh ~/.local/bin/run_sitl ~/.local/bin/kill_sitl
```

Make sure `~/.local/bin` is in your PATH:

```bash
export PATH="$HOME/.local/bin:$PATH"
```

## Start SITL

SITL runs on a laptop by default so Gazebo physics does not overload the
Raspberry Pi. The Pi continues to run the DDS agent, mission executor, and
mission logbook.

One-time setup:

1. Install and build `PX4-Autopilot` on the laptop.
2. Enable an SSH server inside the laptop's Linux/WSL environment and make port
   `22` reachable from the Pi. `run_sitl` expects a Linux shell, not Windows
   PowerShell.
3. Configure key-based SSH access from the Pi.
4. Create the Pi-side configuration:

```bash
mkdir -p ~/.config/ros_px4
cp ~/ROS_PX4/config/sitl.env.example ~/.config/ros_px4/sitl.env
nano ~/.config/ros_px4/sitl.env
```

Set `SITL_HOST` to the laptop's SSH target and `SITL_AGENT_IP` to the Pi's IP
address visible from the laptop. Test the connection from the Pi:

```bash
source ~/.config/ros_px4/sitl.env
ssh "$SITL_HOST" 'test -d ~/PX4-Autopilot && echo ready'
```

Then launch from the Pi:

```bash
run_sitl
```

The Pi's tmux session displays the remote PX4 terminal alongside the local DDS
agent, mission executor, and mission logbook. PX4's remote DDS client is
automatically reconnected to the Pi on UDP port `8888`. No physical drone or
QGroundControl connection is required.

Run PX4 and Gazebo locally on the Pi only as a fallback:

```bash
SITL_MODE=local run_sitl
```

In the PX4 `pxh>` pane, start the simulated mission:

```bash
commander mode offboard
commander arm
```

The executor first streams and holds the initial takeoff setpoint. It starts the
mission timer only after simulated PX4 is armed, in Offboard mode, within
`0.12 m` of that setpoint, and moving slower than `0.15 m/s` for `1.5 s`.

The logbook automatically records mission events and state samples at `100 Hz`,
including mode, arming, failsafe, landed state, position, and velocity. Change
the requested rate with `LOGBOOK_RATE_HZ`; summaries report the achieved rate.
New filenames include a readable timestamp,
`sitl`, and the mission name:

```text
2026-06-12_14-05-30__sitl__mission_sitl_test__events.csv
2026-06-12_14-05-30__sitl__mission_sitl_test__samples.csv
2026-06-12_14-05-30__sitl__mission_sitl_test__commands.csv
2026-06-12_14-05-30__sitl__mission_sitl_test__summary.txt
2026-06-12_14-05-30__sitl__mission_sitl_test__trajectory_3d.png
2026-06-12_14-05-30__sitl__mission_sitl_test__xy.png
2026-06-12_14-05-30__sitl__mission_sitl_test__height.png
```

The 3D plot overlays numbered mission waypoints, the streamed trajectory
commands, and the recorded drone path.
The top-down XY and height plots compare the streamed trajectory setpoints with
the recorded position during the armed flight interval, excluding ground
estimator drift before takeoff and after disarm. The commands CSV captures every
streamed trajectory setpoint sent to PX4. These artifacts are regenerated when
the logbook saves, including during graceful cleanup.

Give an experiment a descriptive run name with:

```bash
RUN_NAME=wind_test_01 run_sitl
RUN_NAME=controller_gain_a MISSION_FILE=$HOME/ROS_PX4/missions/robust_control.tsv run_sitl
LOGBOOK_RATE_HZ=100 RUN_NAME=high_rate_test run_sitl
```

When `RUN_NAME` is omitted, it defaults to the mission filename.

Use the cleanup command when finished so the logbook receives a graceful stop
and writes its buffered data.

## Run Against PX4 Hardware

After the PX4 DDS agent and vehicle connection are available:

```bash
cd ~/ROS_PX4
colcon build
START_ENCODER=true scripts/run_dds_mission.sh missions/mission_sitl_test.tsv
```

The launcher starts the mission executor, independent safety monitor, flight
logger, and payload encoder in a tmux session. Set `START_ENCODER=false` when
running without the Raspberry Pi encoder hardware.

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
MISSION_FILE=$HOME/ROS_PX4/missions/mission_sitl_done_test.tsv run_sitl
```

## Mission logbook

`run_sitl` starts the logbook automatically. Logs save to:

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
t type mode profile x y z vx vy vz ax heading_deg
```

Coordinate convention:

- Mission `x`: north/forward
- Mission `y`: east/right
- Mission `z`: positive up
- PX4 NED `z`: positive down, so the executor converts mission `z` to `-z`
- Optional `heading_deg`: compass heading in degrees from north, increasing
  clockwise (`0=north`, `90=east`, `180=south`, `270=west`)

Example:

```text
t	type	mode	profile	x	y	z	vx	vy	vz	ax	heading_deg
0	takeoff	pos	hold	0	0	1.0	0	0	0	0	0
5	wp	pos	hold	1	0	1.0	0	0	0	0	90
10	end	end	end	0	0	0	0	0	0	0
```

Use `profile=linear` on consecutive position rows to interpolate the target
between waypoints. Use `profile=hold` to hold each target until the next row.
Heading is interpolated over linear position segments using the shortest turn.

The SITL launcher lands automatically at mission end. Because some Gazebo/PX4
combinations do not assert landed state reliably, SITL force-disarms 15 seconds
after issuing LAND. This forced timeout is not enabled by the hardware launcher.

## Cleanup

```bash
kill_sitl
```

This saves the active mission logbook before stopping PX4 and Gazebo.
