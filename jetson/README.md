# PX4 ROS2 DDS SITL Mission Executor

This repo contains a ROS2 Humble PX4 Offboard mission executor, a mission logbook monitor, SITL startup scripts, and example TSV mission files.

## Features

- Publishes PX4 Offboard heartbeat to `/fmu/in/offboard_control_mode`
- Publishes trajectory setpoints to `/fmu/in/trajectory_setpoint`
- Provides a terminal mission-control UI for Offboard, Arm, mission selection,
  Takeoff, Land, RTL, and Disarm
- Holds current local position until Takeoff is selected, then flies the first
  TSV setpoint and starts the mission timer after settling
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
  Default.tsv
  square.tsv
```

## Install into an existing ROS2 workspace

Copy the package into your workspace:

```bash
mkdir -p ~/ros2_ws/src
ln -sfn ~/ROS_PX4/shared/zed_px4_bridge_folder ~/ros2_ws/src/zed_px4_bridge
cd ~/ros2_ws
colcon build --packages-select zed_px4_bridge --symlink-install
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Copy scripts and mission files:

```bash
cp scripts/start_px4_sitl_mission.sh ~/start_px4_sitl_mission.sh
ln -sf ~/ROS_PX4/jetson/scripts/run_sitl ~/.local/bin/run_sitl
ln -sf ~/ROS_PX4/jetson/scripts/run_sitl_105 ~/.local/bin/run_sitl_105
ln -sf ~/ROS_PX4/jetson/scripts/kill_sitl.sh ~/.local/bin/kill_sitl
ln -sf ~/ROS_PX4/jetson/scripts/kill_sitl_105 ~/.local/bin/kill_sitl_105
cp missions/Default.tsv ~/Default.tsv

chmod +x ~/start_px4_sitl_mission.sh \
  ~/ROS_PX4/jetson/scripts/run_sitl \
  ~/ROS_PX4/jetson/scripts/run_sitl_105 \
  ~/ROS_PX4/jetson/scripts/kill_sitl.sh \
  ~/ROS_PX4/jetson/scripts/kill_sitl_105
```

Make sure `~/.local/bin` is in your PATH:

```bash
export PATH="$HOME/.local/bin:$PATH"
```

## Start SITL

SITL runs on a laptop by default so Gazebo physics does not overload the
Jetson. The Jetson continues to run the DDS agent, mission executor, and
mission logbook.

One-time setup:

1. Install and build `PX4-Autopilot` on the laptop.
2. Enable an SSH server inside the laptop's Linux/WSL environment and make port
   `22` reachable from the Jetson. `run_sitl` expects a Linux shell, not Windows
   PowerShell.
3. Configure key-based SSH access from the Jetson.
4. Create the Jetson-side configuration:

```bash
mkdir -p ~/.config/ros_px4
cp ~/ROS_PX4/jetson/config/sitl.env.example ~/.config/ros_px4/sitl.env
cp ~/ROS_PX4/jetson/config/sitl-105.env.example ~/.config/ros_px4/sitl-105.env
nano ~/.config/ros_px4/sitl.env
```

Set `SITL_HOST` to the laptop's SSH target and `SITL_AGENT_IP` to the Jetson's
IP address visible from the laptop. Test the connection from the Jetson:

```bash
source ~/.config/ros_px4/sitl.env
ssh "$SITL_HOST" 'test -d ~/PX4-Autopilot && echo ready'
```

For the second PC profile, add this alias on the Jetson:

```text
Host pc-105-sitl
    HostName 192.168.0.105
    Port 2222
    User lsuic
```

Then test and launch that profile with:

```bash
ssh pc-105-sitl 'test -d ~/PX4-Autopilot && echo ready'
run_sitl_105
```

Then launch from the Jetson:

```bash
run_sitl
```

The Jetson's tmux session displays the remote PX4 terminal alongside the local DDS
agent, mission executor, and mission logbook. PX4's remote DDS client is
automatically reconnected to the Jetson on UDP port `8888`. No physical drone or
QGroundControl connection is required.

Run PX4 and Gazebo locally on the Jetson only as a fallback:

```bash
SITL_MODE=local run_sitl
```

Use the terminal mission-control UI pane to start the simulated mission:

1. Select `Switch Offboard`.
2. Select `Arm`.
3. Optionally select a different `.tsv` with `Select Mission`.
4. Select `Takeoff / Start TSV`.
5. Select `Kill SITL` when you want to stop PX4, Gazebo, the DDS agent, and the
   tmux session.

`Select Mission` lists `.tsv` files from `MISSION_DIR`, which defaults to
`~/ROS_PX4/shared/missions`. For example:

```bash
MISSION_DIR=$HOME/ROS_PX4/shared/missions RUN_NAME=hover_test run_sitl
```

Before takeoff is selected, the executor streams a hold at the current local
position so Offboard remains available without commanding the first airborne
waypoint. It starts the mission timer only after simulated PX4 is armed, in
Offboard mode, takeoff has been requested, the vehicle is within `0.12 m` of the
first TSV setpoint, and it is moving slower than `0.15 m/s` for `1.5 s`.

The launcher also starts a browser mission-control panel on the Jetson:

```text
http://JETSON_IP:8080
```

Set `MISSION_WEB_PORT` to use a different port. The terminal UI remains
available in tmux as a fallback.

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
2026-06-12_14-05-30__sitl__mission_sitl_test__x_time.png
2026-06-12_14-05-30__sitl__mission_sitl_test__y_time.png
```

The 3D plot overlays numbered mission waypoints, the streamed trajectory
commands, and the recorded drone path.
The top-down XY and height plots compare the streamed trajectory setpoints with
the recorded position during the armed flight interval. The X-vs-time and
Y-vs-time plots show each horizontal axis separately against the streamed
setpoints. These plots exclude ground estimator drift before takeoff and after
disarm. The commands CSV captures every streamed trajectory setpoint sent to
PX4. These artifacts are regenerated when the logbook saves, including during
graceful cleanup.

Give an experiment a descriptive run name with:

```bash
RUN_NAME=wind_test_01 run_sitl
RUN_NAME=controller_gain_a MISSION_FILE=$HOME/ROS_PX4/shared/missions/robust_control.tsv run_sitl
LOGBOOK_RATE_HZ=100 RUN_NAME=high_rate_test run_sitl
```

When `RUN_NAME` is omitted, it defaults to the mission filename.

Use the cleanup command when finished so the logbook receives a graceful stop
and writes its buffered data.

## Run Against PX4 Hardware

For the complete preflight, launch, recovery, shutdown, and logging procedure,
see [Real Drone Mission Guide](REAL_DRONE_MISSION.md).

After the PX4 DDS agent and vehicle connection are available:

```bash
cd ~/ROS_PX4/jetson
colcon build
START_ENCODER=true scripts/run_dds_mission.sh
```

The launcher starts the mission executor, independent safety monitor, flight
logger, and payload encoder in a tmux session. Set `START_ENCODER=false` when
running without the Jetson encoder hardware.

For this ARK PAB Jetson, QGroundControl MAVLink is handled by
`mavlink-router.service` over the Pixhawk USB-C/FCUSB link:

```bash
~/ROS_PX4/jetson/tools/set_qgc_target.sh
systemctl --user status mavlink-router.service
```

Then use the terminal mission-control UI pane to select `Switch Offboard`,
`Arm`, and `Takeoff / Start TSV`.

The hardware launcher also starts the browser mission-control panel at
`http://JETSON_IP:8080`. Use it for the same controls and live status, while
keeping the terminal UI as a fallback.

At mission end, the drone should hold the final setpoint. Land manually:

```bash
commander land
```

## Nano MT6835 angle readings and flight logs

The Arduino Nano reads both MT6835 encoders over SPI and streams their angles
over USB serial. The ROS `mt6835_encoder` node forwards those readings to the
flight logger. While PX4 is in Offboard, the logger saves them to a separate
`encoder_angles.csv` inside that run's log folder.

### Prepare the Nano

Open `~/ch341-driver/mt6835_dual_encoder.ino` in Arduino IDE, select the Nano
and its USB port, and upload the sketch. The IDE is only needed to upload or
change the Arduino sketch; you can calibrate the zero point from the Jetson
terminal. Close Serial Monitor before using the Jetson serial tools because
only one program can use the serial port at a time.

### Set up the Jetson serial port and ROS node

The Jetson's current kernel may need the locally built CH341 driver loaded
before the Nano appears as `/dev/ttyUSB0`:

```bash
sudo modprobe usbserial
sudo modprobe ch341 2>/dev/null || sudo insmod "$HOME/ch341-driver/ch341.ko"
ls -l /dev/ttyUSB0
```

`insmod: ... File exists` is harmless and means the driver is already loaded.

Install pyserial and rebuild the ROS package after pulling these changes:

```bash
sudo apt install python3-serial
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select zed_px4_bridge --symlink-install
```

If `brltty` is installed and the serial device repeatedly disappears, it may
be claiming the CH340 adapter. Stop or configure `brltty` not to probe USB ID
`1a86:7523`.

### View live angles from the Jetson terminal

After `/dev/ttyUSB0` exists, start the serial reader with:

```bash
python3 ~/ch341-driver/read_angles.py
```

It prints both angles continuously at the Nano's 115200 baud rate. Move the
encoders to any position, then type `z` and press Enter to make that position
zero for both encoders. Hold the encoders still until `Zero saved for both
encoders.` appears. The zero offsets are stored in the Nano's EEPROM and remain
active after a restart. Calibration averages multiple readings, and the small
sensor-noise region around the saved reference is reported as exactly `0.000`
instead of wrapping to nearly `360`. Type `q` and Enter or press `Ctrl+C` to
stop the reader.

To zero immediately after connecting, first hold both encoders at the desired
zero pose, then run:

```bash
python3 ~/ch341-driver/read_angles.py --zero
```

The Arduino IDE Serial Monitor, calibration utility, and ROS flight logger
cannot use the same serial port at the same time, so stop this reader before
running any of those.

### Calibrate without Arduino IDE

Stop the live angle reader if it is running, then run the calibration utility:
sudo modprobe usbserial
sudo insmod "$HOME/ch341-driver/ch341.ko"
ls -l /dev/ttyUSB0

```bash
python3 ~/ch341-driver/calibrate.py
```

When prompted, move both encoders to a known reference pose and press Enter.
Enter the angle that each encoder should report at that pose; pressing Enter
for an encoder uses `0` degrees. For example, entering `45` for encoder 1 and
`90` for encoder 2 makes the current pose read 45 and 90 degrees. The utility
sends `c,45,90` to the Nano and expects a calibration confirmation. Values are
wrapped to one revolution, and offsets are stored in EEPROM. The uploaded
sketch must support the `c,<angle1>,<angle2>` command. Do not run the utility
while the ROS mission logger or another serial monitor is using `/dev/ttyUSB0`.

The equivalent Arduino Serial Monitor command is `c,45,90` followed by a
newline. A bare `c` sets both current positions to zero.

### Record a selected mission

Follow [Real Drone Mission Guide](REAL_DRONE_MISSION.md) for the complete
preflight, bench-test, recovery, and shutdown requirements. The steps below
start a real PX4 mission and record the Nano MT6835 encoder angles.

1. Confirm the PX4 DDS link and local position are valid, QGroundControl shows
  live vehicle data, and the RC transmitter and recovery controls are ready.
  Do not fly if local position is invalid or QGroundControl reports a warning
  you have not resolved.
2. Calibrate before starting the mission launcher. No Arduino Serial Monitor,
  ROS MT6835 node, or other serial reader may be using `/dev/ttyUSB0`. If a
  mission session is already running, land and disarm before stopping it:

  ```bash
  ~/ROS_PX4/jetson/scripts/stop_dds_mission.sh
  python3 ~/ch341-driver/calibrate.py
  ```

  Place both encoders at the reference pose and enter the angle each should
  report. Pressing Enter without entering an angle uses `0` degrees. To set the
  current pose directly to zero for both encoders instead, run:

  ```bash
  python3 ~/ch341-driver/read_angles.py --zero
  ```

  Wait for confirmation and ensure the utility exits before continuing. The
  calibration is stored in the Nano's EEPROM. Never calibrate during flight.
3. Review the mission file and confirm its path and trajectory are appropriate
  for the flight area. For example:

  ```bash
  MISSION_FILE="$HOME/ROS_PX4/shared/missions/straight_line_mission.tsv"
  column -s $'\t' -t < "$MISSION_FILE"
  ```

  The example `straight_line_mission.tsv` currently commands a 10 m altitude.
  The hardware safety monitor defaults to a 2 m maximum altitude, 1.5 m/s
  maximum velocity, and 5 m local radius. Review the planned trajectory and
  configure the safety limits for the approved test envelope before launch;
  do not blindly copy example limits from elsewhere in this guide. Exceeding
  an abort limit requests RTL.
4. Connect the Nano and confirm its serial device exists. The default is
  `/dev/ttyUSB0`:

  ```bash
  ls -l /dev/ttyUSB0
  ```

  If you use `read_angles.py` to check or zero the encoders, exit it before
  launching the mission. The ROS bridge, Arduino Serial Monitor, calibration
  utility, and live reader cannot use the serial port at the same time.
5. If this is the first run after updating the package, install `python3-serial`
  and rebuild it as described in [Set up the Jetson serial port and ROS node](#set-up-the-jetson-serial-port-and-ros-node).
6. From the Jetson, launch the desired TSV with the Nano bridge enabled:

```bash
cd ~/ROS_PX4/jetson
  export MISSION_FILE="$HOME/ROS_PX4/shared/missions/straight_line_mission.tsv"
  export MISSION_DIR="$HOME/ROS_PX4/shared/missions"
  START_ENCODER=false \
  START_MT6835_ENCODER=true \
  MT6835_SERIAL_PORT=/dev/ttyUSB0 \
  scripts/run_dds_mission.sh "$MISSION_FILE"
```

  `START_MT6835_ENCODER=true` enables the Nano bridge. `START_ENCODER` controls
  a separate quadrature encoder, so leave it `false` unless that hardware is
  connected. The Nano bridge defaults to `/dev/ttyUSB0` at 115200 baud; use
  `MT6835_SERIAL_PORT` or `MT6835_BAUD_RATE` to override those settings.
7. In a second SSH terminal, verify that both encoder angles are being
  published before arming:

  ```bash
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  ros2 topic echo --no-daemon \
    /mt6835/angles std_msgs/msg/Float64MultiArray \
    --qos-profile sensor_data
  ```

  Move each encoder separately and confirm that the corresponding value in the
  two-element `data` array changes. Press `Ctrl+C` only in this verification
  terminal. Do not start another serial reader while the mission bridge owns
  `/dev/ttyUSB0`.
8. Wait for the executor to report that the mission is loaded and local
  position is available. Check QGroundControl for warnings. The terminal
  mission UI is available in tmux; the browser UI is at
  `http://JETSON_IP:8080`.
9. When the pilot is ready, use either UI to select **Switch Offboard**, then
  **Arm**. The executor begins streaming a current-position hold; arming does
  not start the TSV. Select **Select Mission** if you need to change the file,
  then select **Takeoff / Start TSV** to begin the mission. Keep the pilot at
  the controls and monitor the vehicle, QGroundControl, and mission UI.
10. The logger begins recording Nano encoder samples when PX4 confirms Offboard.
  It stops recording and durably flushes `encoder_angles.csv` when the mission
  completes, aborts, or PX4 leaves Offboard. At mission end, land with
  QGroundControl or the RC transmitter; the hardware launcher does not land or
  disarm automatically. After landing and disarming, select **Shutdown / Save
  Logs** in the UI to finalize metadata, plots, and the PX4 ULog copy.

Logs are saved under `~/logs/<timestamp>_real__<mission>/`. To view the newest
run's angle log after stopping the mission:

```bash
latest_run="$(ls -td "$HOME"/logs/*/ | head -n 1)"
wc -l "$latest_run/encoder_angles.csv"
tail "$latest_run/encoder_angles.csv"
grep 'encoder_angle_rows' "$latest_run/metadata.json"
column -s, -t < "$latest_run/encoder_angles.csv" | less -S
```

The CSV contains wall-clock time, elapsed run time, encoder 1 angle in
degrees, and encoder 2 angle in degrees. Its line count must be greater than one
because the first line is the header. **Shutdown / Save Logs** uses the stop
helper, which waits for the logger to flush `encoder_angles.csv`, write its
sample count to `metadata.json`, and finish the remaining run output before
closing tmux. Use `q` to exit the viewer. The existing `START_ENCODER` option
controls the separate quadrature payload encoder and does not enable or disable
the Nano bridge.

## Use a short test mission

```bash
MISSION_FILE=$HOME/ROS_PX4/shared/missions/Default.tsv run_sitl
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
5	wp	pos	linear	1	0	1.0	0	0	0	0	90
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
## SITL on this Windows PC

The `sitl-173` profile runs PX4 and Gazebo in this PC's WSL Ubuntu while the
Jetson at `192.168.0.170` runs the DDS agent, mission executor, and logbook.
The profile is in `jetson/config/sitl-173.env.example`. It opens the Gazebo
GUI on this Windows PC through WSLg.

PC prerequisites: install and build `~/PX4-Autopilot` with Gazebo in WSL
Ubuntu, run an SSH server in WSL, expose it to the Jetson at
`DESKTOP-BGT94JT.local:2222`, and authorize the Jetson's SSH key. Allow the PC's PX4
DDS client to reach `192.168.0.170:8888/udp`. The profile alone does not
install these PC components.

On the Jetson, copy the profile:

```bash
mkdir -p ~/.config/ros_px4
cp ~/ROS_PX4/jetson/config/sitl-173.env.example ~/.config/ros_px4/sitl-173.env
```

Add this SSH alias to the Jetson's `~/.ssh/config`:

```text
Host pc-173-sitl
    HostName DESKTOP-BGT94JT.local
    Port 2222
    User darren_johnson
```

Check connectivity, then start and stop the mission with the profile:

```bash
ssh -o BatchMode=yes pc-173-sitl 'test -d ~/PX4-Autopilot && echo ready'
bash ~/ROS_PX4/shared/scripts/run_sitl_173
bash ~/ROS_PX4/shared/scripts/kill_sitl_173
```

The `bash` form works without changing script permissions. If either device's
Jetson LAN address changes, update `SITL_AGENT_IP` in the copied profile
before launching. The PC alias follows its current LAN address through mDNS.
