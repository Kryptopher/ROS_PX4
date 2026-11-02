# Real Drone Mission Guide

This guide runs a TSV mission on the physical PX4 vehicle using ROS 2 on the
Jetson. It does not start PX4 SITL or Gazebo.

> **Flight safety:** Remove the propellers for setup and bench testing. For the
> first powered flight, use a short conservative hover mission in a clear
> outdoor area with a pilot holding the RC transmitter. Confirm that the RC
> mode switch, Land, RTL, and emergency stop behavior work before using
> Offboard mode. Follow local operating rules and the vehicle's normal
> preflight checklist.

---

## How the hardware is connected

* ARK PAB TELEM2 carries uXRCE-DDS data to the Jetson for ROS 2.
* The Pixhawk USB-C connection carries MAVLink to the Jetson.
* `mavlink-router.service` forwards USB MAVLink from the Pixhawk to
  QGroundControl over UDP port `14550`.
* The mission launcher runs the executor, safety monitor, flight logger, and
  optional payload encoder on the Jetson.

This vehicle uses the ARK PAB / PX4 FMUv6X-style flight controller stack. In
QGroundControl, assign uXRCE-DDS to `TELEM2`. The cable connects TELEM2 TX to
Jetson RX, TELEM2 RX to Jetson TX, and ground to ground. On this Jetson, the
selected UART is `/dev/ttyTHS1`.

Leave the Pixhawk USB-C cable connected. USB-C remains dedicated to MAVLink and
`mavlink-router`, while TELEM2 is dedicated to uXRCE-DDS.

Recommended link split:

```text
Pixhawk USB-C  -> Jetson USB       -> MAVLink / QGroundControl router
Pixhawk TELEM2 -> Jetson UART      -> uXRCE-DDS / ROS 2 Offboard
Jetson Wi-Fi   -> Laptop/QGC/SSH   -> operator connection
```

---

## Jetson network modes

The Jetson supports two operator network modes:

```text
HomeWifi mode:
  Used for bench testing and configuration.
  Jetson connects to the normal Wi-Fi network.
  Laptop/QGC is also on the normal Wi-Fi network.

JetsonHotspot mode:
  Used in the field.
  Jetson broadcasts its own Wi-Fi network.
  Laptop/QGC connects directly to the Jetson hotspot.
```

The Jetson's Wi-Fi interface is:

```bash
wlP1p1s0
```

The hotspot settings are:

```text
SSID:     JetsonDrone
Password: JetsonDrone
Jetson IP in hotspot mode: 10.42.0.1
```

The Jetson usually cannot reliably connect to `HomeWifi` and broadcast
`JetsonHotspot` at the same time with the built-in Wi-Fi adapter. Use one mode
at a time.

---

## One-time Jetson setup

Build the ROS 2 package after cloning or updating the repository:

```bash
cd ~/ROS_PX4/jetson
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The launcher sources the existing `~/ros2_ws/install/setup.bash`. Install
the package into that workspace:

```bash
mkdir -p ~/ros2_ws/src
ln -sfn ~/ROS_PX4/shared/zed_px4_bridge_folder ~/ros2_ws/src/zed_px4_bridge
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select zed_px4_bridge --symlink-install
```

---

## One-time Wi-Fi setup

### Confirm the Wi-Fi device

```bash
nmcli device status
```

Expected Wi-Fi device:

```text
wlP1p1s0
```

### Configure HomeWifi

If the home Wi-Fi is hidden, use the hidden-network setup below. Replace the
SSID and password with the real values.

```bash
sudo nmcli radio wifi on

sudo nmcli connection add type wifi ifname wlP1p1s0 con-name HomeWifi ssid "YOUR_HIDDEN_WIFI_NAME"

sudo nmcli connection modify HomeWifi \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "YOUR_WIFI_PASSWORD" \
  802-11-wireless.hidden yes \
  connection.autoconnect yes

sudo nmcli connection up HomeWifi
```

Confirm the Jetson is connected:

```bash
nmcli device status
ip -4 addr show wlP1p1s0
ping -c 4 1.1.1.1
```

Find the Jetson's HomeWifi IP:

```bash
ip -4 addr show wlP1p1s0
```

Example:

```text
inet 192.168.0.167/24
```

From the laptop, SSH to the Jetson using that IP:

```bash
ssh scs@192.168.0.167
```

### Configure JetsonHotspot

Create one clean hotspot profile:

```bash
sudo nmcli connection add type wifi ifname wlP1p1s0 con-name JetsonHotspot ssid JetsonDrone

sudo nmcli connection modify JetsonHotspot \
  802-11-wireless.mode ap \
  802-11-wireless.band bg \
  ipv4.method shared \
  ipv4.addresses 10.42.0.1/24 \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "JetsonDrone" \
  connection.autoconnect no
```

Confirm there is only one `JetsonHotspot` profile:

```bash
nmcli connection show
```

If there are duplicate `JetsonHotspot` entries, delete the extra one by UUID:

```bash
sudo nmcli connection delete UUID_TO_DELETE
```

---

## Switching Wi-Fi modes manually

### HomeWifi mode

Use this for bench testing and configuration on the normal Wi-Fi network:

```bash
sudo nmcli connection down JetsonHotspot 2>/dev/null
sudo nmcli connection up HomeWifi
nmcli device status
ip -4 addr show wlP1p1s0
```

Expected:

```text
wlP1p1s0 connected HomeWifi
inet 192.168.0.xxx/24
```

SSH from the laptop:

```bash
ssh scs@JETSON_HOME_WIFI_IP
```

Example:

```bash
ssh scs@192.168.0.167
```

### JetsonHotspot mode

Use this in the field:

```bash
sudo nmcli connection down HomeWifi 2>/dev/null
sudo nmcli connection up JetsonHotspot
nmcli device status
ip -4 addr show wlP1p1s0
```

Expected:

```text
wlP1p1s0 connected JetsonHotspot
inet 10.42.0.1/24
```

Connect the laptop to:

```text
Wi-Fi: JetsonDrone
Password: JetsonDrone
```

SSH from the laptop:

```bash
ssh scs@10.42.0.1
```

---

## Optional automatic Wi-Fi fallback

This optional service tries `HomeWifi` at boot. If no internet is detected
within 15 seconds, it switches to `JetsonHotspot`.

Create the script:

```bash
sudo tee /usr/local/bin/drone-network-autoswitch.sh >/dev/null <<'SH'
#!/usr/bin/env bash
set -u

WIFI_IF="wlP1p1s0"
HOME_CON="HomeWifi"
HOTSPOT_CON="JetsonHotspot"
WAIT_SECONDS=15

log() {
  logger -t drone-network-autoswitch "$*"
  echo "[drone-network-autoswitch] $*"
}

internet_ok() {
  ping -I "$WIFI_IF" -c 1 -W 2 1.1.1.1 >/dev/null 2>&1 || \
  ping -I "$WIFI_IF" -c 1 -W 2 8.8.8.8 >/dev/null 2>&1
}

log "Starting network auto-switch."

nmcli radio wifi on || true

log "Stopping hotspot if active."
nmcli connection down "$HOTSPOT_CON" >/dev/null 2>&1 || true

log "Trying HomeWifi."
nmcli connection up "$HOME_CON" >/dev/null 2>&1 || true

for i in $(seq 1 "$WAIT_SECONDS"); do
  if internet_ok; then
    IP_ADDR="$(ip -4 addr show "$WIFI_IF" | awk '/inet / {print $2}' | head -1)"
    log "Internet detected on $HOME_CON. Staying on home Wi-Fi. IP: ${IP_ADDR:-unknown}"
    exit 0
  fi
  sleep 1
done

log "No internet after ${WAIT_SECONDS}s. Switching to hotspot."

nmcli connection down "$HOME_CON" >/dev/null 2>&1 || true
sleep 2
nmcli connection up "$HOTSPOT_CON"

IP_ADDR="$(ip -4 addr show "$WIFI_IF" | awk '/inet / {print $2}' | head -1)"
log "Hotspot should now be active. IP: ${IP_ADDR:-unknown}"

exit 0
SH

sudo chmod +x /usr/local/bin/drone-network-autoswitch.sh
```

Create the systemd service:

```bash
sudo tee /etc/systemd/system/drone-network-autoswitch.service >/dev/null <<'EOF'
[Unit]
Description=Drone Jetson Wi-Fi Home/Hotspot Auto Switch
Wants=NetworkManager.service
After=NetworkManager.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/drone-network-autoswitch.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
```

Enable it:

```bash
sudo systemctl daemon-reload
sudo systemctl enable drone-network-autoswitch.service
```

Test it without rebooting:

```bash
sudo systemctl start drone-network-autoswitch.service
sudo systemctl status drone-network-autoswitch.service
nmcli device status
ip -4 addr show wlP1p1s0
```

View logs:

```bash
journalctl -u drone-network-autoswitch.service -n 80 --no-pager
```

---

## QGroundControl MAVLink connection

The Pixhawk USB-C cable carries MAVLink to the Jetson. The Jetson forwards that
MAVLink stream to QGroundControl over UDP port `14550`.

This ARK setup uses `mavlink-router` in UDP server mode, so the service does
not need a hardcoded laptop IP. QGroundControl can connect from the Jetson
hotspot or from normal Wi-Fi as long as it can reach the Jetson.

Install or refresh the ARK MAVLink router config:

```bash
chmod +x ~/ROS_PX4/jetson/tools/set_qgc_target.sh
~/ROS_PX4/jetson/tools/set_qgc_target.sh
```

Despite the historical name, `set_qgc_target.sh` now installs the ARK
`mavlink-router` service and config. It does not prompt for a laptop IP.

In QGroundControl, use UDP on port `14550`. In hotspot mode, connect to:

```text
10.42.0.1:14550
```

### Refresh the QGC router command

Run this whenever you want to reinstall the ARK MAVLink router service/config:

```bash
~/ROS_PX4/jetson/tools/set_qgc_target.sh
```

For HomeWifi mode, QGC should connect to the Jetson's HomeWifi IP, usually:

```text
192.168.0.xxx
```

For JetsonHotspot mode, QGC should connect to the Jetson hotspot IP:

```text
10.42.0.1:14550
```

To find connected hotspot clients from the Jetson:

```bash
ip neigh
```

Example output:

```text
10.42.0.23 dev wlP1p1s0 lladdr xx:xx:xx:xx:xx:xx REACHABLE
```

```text
10.42.0.23
```

QGroundControl should use UDP auto-connect on port `14550`. Do not create a
second manual QGC link on the same port.

---

## QGC router check

Confirm the router is running:

```bash
systemctl --user status mavlink-router.service
```

Show the current service:

```bash
systemctl --user cat mavlink-router.service
```

Restart the router:

```bash
systemctl --user restart mavlink-router.service
```

---

## Jetson R36.5 UART fix

This Jetson's L4T R36.5 device tree shipped with broken DMA properties for
UART1, causing null bytes and truncated serial data. The fix is already
installed on this vehicle and forces `/dev/ttyTHS1` to reliable PIO mode. If
the boot files are replaced by an OS update and the UART loopback regresses,
reapply it with:

```bash
sudo ~/ROS_PX4/jetson/tools/install_jetson_r36_5_uart_fix.sh
sudo reboot
```

---

## DDS agent setup

Install and enable the ARK PAB flight controller TELEM2 DDS agent service:

```bash
mkdir -p ~/.config/systemd/user
install -m 0644 ~/ROS_PX4/jetson/config/dds-agent.service \
  ~/.config/systemd/user/dds-agent.service
systemctl --user daemon-reload
systemctl --user enable --now dds-agent.service
systemctl --user status dds-agent.service
```

This service runs `MicroXRCEAgent` on the Jetson's `/dev/ttyTHS1` UART at
3,000,000 baud. Confirm the physical TELEM2 cable is connected to that UART
before expecting DDS topics.

Configure the matching PX4 side once in QGC Parameters:

```text
UXRCE_DDS_CFG = TELEM 2
SER_TEL2_BAUD = 3000000
```

Some newer PX4 builds expose `UXRCE_DDS_FLCTRL`. It is not present on this
vehicle's firmware and is not required for the three-wire TX/RX/ground link.

Ensure no `MAV_*_CONFIG` or other serial driver is assigned to TELEM2. Reboot
PX4 with the propellers removed. In the QGC MAVLink Console, verify:

```text
uxrce_dds_client status
```

It should report `Running, connected`. QGC continues to use the separate USB-C
MAVLink router.

If the relayed QGC MAVLink Console is blank, verify DDS directly on the Jetson:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo --once --qos-reliability best_effort \
  --qos-durability transient_local /fmu/out/sensor_combined
```

A live sample confirms the PX4 client, TELEM2 link, Jetson agent, and ROS 2 DDS
path are all working even if the console does not render the status response.

---

## 1. Bench test with propellers removed

Power the vehicle normally and connect the Pixhawk to the Jetson. Check the DDS
agent:

```bash
systemctl --user status dds-agent.service
```

Confirm that PX4 data is arriving:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep '^/fmu/'
ros2 topic echo --once --qos-reliability best_effort \
  --qos-durability transient_local /fmu/out/vehicle_status_v1
ros2 topic echo --once --qos-reliability best_effort \
  --qos-durability transient_local /fmu/out/vehicle_local_position_v1
```

The local-position message must report valid horizontal and vertical position.
Do not fly if these topics are absent, stale, or invalid.

For an indoor bench test without GPS, do not run the full mission. Use
QGroundControl's motor test page with props removed:

```text
QGroundControl -> Vehicle Setup -> Motors
```

The motor test does not require GPS because it is a bench output test, not a
position-control flight mission.

With the propellers still removed, run the launcher only when local position is
valid:

```bash
cd ~/ROS_PX4/jetson
START_ENCODER=true scripts/run_dds_mission.sh
```

The executor should say that the Offboard stream is idle until the UI requests
it. This prevents the vehicle from entering Offboard just because the launcher
started while a transmitter/QGC mode switch was already set to Offboard.

Do not arm during this basic bench check unless you are deliberately performing
a props-off arming test. Detach from tmux with `Ctrl-B`, then `D`, and stop the
test:

```bash
tmux send-keys -t px4_dds_mission:0.0 C-c
tmux send-keys -t px4_dds_mission:0.1 C-c
tmux send-keys -t px4_dds_mission:0.2 C-c
tmux send-keys -t px4_dds_mission:0.3 C-c
tmux kill-session -t px4_dds_mission
```

---

## 2. Review the mission and safety limits

The included `missions/Default.tsv` mission is a simple hover check:

* Rise to 5 m above the takeoff/start position.
* Hold that position until mission completion.
* Continue holding at mission completion until the pilot lands.

Mission coordinates are relative to PX4's local origin:

* `x`: north/forward
* `y`: east/right
* `z`: positive upward
* `heading_deg`: compass heading clockwise from north

The mission file is tab-delimited. Review it before every flight:

```bash
column -s $'\t' -t missions/Default.tsv
```

The hardware launcher defaults to these independent safety-monitor limits:

| Limit                              | Default |
| ---------------------------------- | ------: |
| Maximum altitude                   |   2.0 m |
| Altitude warning                   |   1.5 m |
| Maximum total velocity             | 1.5 m/s |
| Maximum distance from local origin |   5.0 m |
| DDS local-position timeout         |   2.0 s |

Crossing an abort limit causes the safety monitor and mission executor to
request RTL. These limits supplement PX4's configured geofence and failsafes;
they do not replace them.

Before flight, verify in QGC:

* Airframe, sensors, level horizon, and compass are calibrated.
* GPS and local position are healthy and the home position is correct.
* Battery is suitable for the complete flight plus reserve.
* RC link, flight-mode switch, RTL, Land, and emergency control are available.
* PX4 data-link-loss, Offboard-loss, geofence, and low-battery actions are set
  appropriately for the test site.
* RTL altitude and the return path are safe for the surroundings.

---

## 3. Start the real mission

Place the vehicle at the intended local origin, clear the flight area, and keep
the pilot at the controls.

Make sure the correct Jetson network mode is active.

For HomeWifi bench/config mode:

```bash
sudo nmcli connection down JetsonHotspot 2>/dev/null
sudo nmcli connection up HomeWifi
```

For JetsonHotspot field mode:

```bash
sudo nmcli connection down HomeWifi 2>/dev/null
sudo nmcli connection up JetsonHotspot
```

Connect the QGC laptop to the correct network, then confirm the router is up:

```bash
systemctl --user status mavlink-router.service
```

Confirm QGC shows live attitude, battery, GPS, and arming status.

### Start and verify the MT6835 angle reader

Connect the Arduino Nano before launching the mission. The Nano uses a CH340
USB serial adapter, and this Jetson may need its local CH341 driver loaded after
each reboot:

```bash
sudo modprobe usbserial
sudo modprobe ch341 2>/dev/null || sudo insmod "$HOME/ch341-driver/ch341.ko"
ls -l /dev/ttyUSB0
```

`insmod: ... File exists` is harmless; it means the driver is already loaded.
Do not leave Arduino Serial Monitor, `read_angles.py`, or `calibrate.py` open,
because only one process can own `/dev/ttyUSB0`. The mission launcher starts
the ROS angle reader when `START_MT6835_ENCODER=true`; do not start a second
copy manually.

#### Calibrate the encoders before starting the mission

Calibration must be completed before running the mission launcher because the
calibration utility and ROS angle reader cannot share `/dev/ttyUSB0`. If a
mission session is already running, land and disarm first, then stop it cleanly:

```bash
~/ROS_PX4/jetson/scripts/stop_dds_mission.sh
```

Never stop or calibrate the encoder during flight. With the mission stack
stopped, run the interactive calibration utility:

```bash
python3 ~/ch341-driver/calibrate.py
```

Place both encoders at a known reference pose and follow the prompts. Enter the
angle that each encoder should report at that pose; pressing Enter without a
value uses `0` degrees. To make the current pose zero for both encoders without
the interactive prompts, run:

```bash
python3 ~/ch341-driver/read_angles.py --zero
```

Wait for the calibration confirmation and allow the utility to exit before
launching the mission. The Nano stores the offsets in EEPROM, so they remain
after a reboot or power cycle.

Start the selected flight mission. Set `MISSION_FILE` to the TSV you intend to
fly and choose safety limits that are above the planned mission envelope but
still conservative for the test site:

```bash
cd ~/ROS_PX4
START_ENCODER=false \
START_MT6835_ENCODER=true \
MT6835_SERIAL_PORT=/dev/ttyUSB0 \
MT6835_BAUD_RATE=115200 \
SAFETY_MAX_ALTITUDE_M=30.0 \
SAFETY_WARN_ALTITUDE_M=6.5 \
SAFETY_MAX_VELOCITY_MS=7.5 \
SAFETY_LOCAL_RADIUS_M=30.0 \
jetson/scripts/run_dds_mission.sh "$HOME/ROS_PX4/shared/missions/Default.tsv"
```
`START_ENCODER` controls a separate GPIO quadrature encoder and should remain
`false` for the Nano MT6835 setup. The launcher refuses to continue unless the
serial device exists, is unused, and produces a live encoder sample.

From a second SSH terminal, confirm that both MT6835 angles are arriving before
arming:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo --no-daemon \
  /mt6835/angles std_msgs/msg/Float64MultiArray \
  --qos-profile sensor_data
```

The message contains two values in `data`: encoder 1 and encoder 2 in degrees.
Move each encoder separately and confirm that the corresponding value changes
smoothly. Press `Ctrl+C` only in this verification terminal; keep the mission
launcher and its encoder pane running. Do not arm if no samples arrive.

`square.tsv` flies a 10 m square at 10 m altitude. Each side takes 5 seconds
(2 m/s), and each corner rotates 90 degrees over 6 seconds (15 degrees/s)
while holding position. The mission ends at 57 seconds, including the initial
climb/hold and final hold. These are setpoint rates; actual motion depends on
PX4 tracking. Use safety limits that accommodate 10 m altitude, the far corner
at about 14.2 m from the start, and 2 m/s travel; the launcher's default limits
are below this mission's envelope.

The terminal UI's **Select Mission** option lists `.tsv` files from
`MISSION_DIR`, which defaults to `~/ROS_PX4/shared/missions`. If you select a different
mission in the UI before takeoff, the executor switches to that TSV.

Flight log folders default to a readable label based on the mission name, such
as `20260714_161530_real__hover_5m`. For a special test, add a descriptive
`RUN_LABEL`:

```bash
RUN_LABEL=real__hover_5m_low_gain scripts/run_dds_mission.sh "$MISSION_FILE"
```

Wait until the executor reports that it has loaded the mission and has local
position. A tmux pane opens the terminal mission-control UI, and a browser
mission-control panel is available at:

```text
http://JETSON_IP:8080
```

Use `MISSION_WEB_PORT=8081` or another port if `8080` is already in use. The
terminal UI remains available in tmux as a fallback. Check QGC once more for
warnings.
Then, when the pilot is ready:

1. Select **Switch Offboard** in the browser or terminal UI.
2. Select **Arm** in the browser or terminal UI. This only arms the vehicle; it does not
   start the TSV mission.
3. Select **Select Mission** if you need to change the `.tsv` file before
   takeoff.
4. Select **Takeoff / Start TSV** when you are ready for the aircraft to climb
   to the first TSV setpoint and begin the mission timer.
5. Keep hands on the controls and watch the flight, QGC status, and tmux panes.

Automatic arming and automatic Offboard selection are deliberately disabled.
The executor does not publish the Offboard setpoint stream until **Switch
Offboard** is selected in the UI. After that request, it warms a
current-position hold stream, sends the Offboard mode command, and keeps holding
the current position until **Takeoff / Start TSV** is selected. The mission
clock starts only after takeoff is selected, PX4 is armed, PX4 is in Offboard,
and the vehicle has settled at the first TSV setpoint for 1.5 seconds.

---

## 4. End or abort the flight

At the TSV `end` row, the executor keeps holding the final setpoint in Offboard
until the operator selects **Shutdown / Save Logs**. With the pilot ready on the
RC, select it: the executor requests **Position mode**, keeps streaming until
PX4 leaves Offboard, then stops the mission software and saves logs. The drone
stays armed and hovering under PX4 Position control for manual takeover.
Completion comes from the TSV timer, so planned hovers do not end the mission.

If PX4 rejects Position mode or confirmation is missing, the executor keeps
holding the final Offboard setpoint and leaves the stack running. Take over
using QGC or the RC transmitter. Launch with `AUTO_SHUTDOWN=true` to request
Position mode and save logs automatically at the `end` row instead.

Land using QGC or the RC transmitter when ready. If PX4 leaves Offboard because the pilot
selects Land, RTL, Position, Loiter, or another tested recovery mode, the
executor stops publishing Offboard setpoints so the transmitter/QGC mode can
take over cleanly.

If anything looks wrong, the pilot should immediately use the safest available
recovery action for the situation:

* Switch out of Offboard into a tested manual/position mode to take control.
* Select Land when landing in place is safest.
* Select RTL only when the home position, altitude, and return path are safe.
* Use the emergency stop/kill function only for an actual emergency and only
  with full awareness that thrust will stop immediately.

Do not kill the executor as the normal landing method while the vehicle is still
in Offboard. Use a flight-mode change or the UI's Land/RTL controls so PX4 gets
a deliberate command and the executor can relinquish control cleanly.

---

## 5. Shut down and collect logs

After the operator selects **Shutdown / Save Logs** and PX4 leaves Offboard, the
executor calls the shutdown helper. It stops the mission panes, lets the logger
finalize its files, and closes the tmux session. It does not land, disarm, or
power off the Jetson. PX4 keeps flying in Position mode, and the pilot must take
over and land manually. If PX4 is not armed in Offboard, the helper runs at once.

Rebuild the ROS package using the setup instructions above before running the
updated launcher.

If the UI is not available, run the shutdown helper from another terminal only
after landing or once the pilot has control. It skips the Position mode handover
and stops the Offboard stream immediately, so it asks for confirmation:

```bash
~/ROS_PX4/jetson/scripts/stop_dds_mission.sh
```

Manual fallback:

```bash
tmux send-keys -t px4_dds_mission:0.0 C-c
tmux send-keys -t px4_dds_mission:0.1 C-c
tmux send-keys -t px4_dds_mission:0.2 C-c
tmux send-keys -t px4_dds_mission:0.3 C-c
sleep 2
tmux kill-session -t px4_dds_mission
```

Hardware-flight logs are written under `~/logs/`. Find the newest run with:

```bash
ls -td ~/logs/* | head -1
```

Each run contains `flight.csv`, `events.csv`, `commands.csv`,
`encoder_angles.csv`, `metadata.json`, `summary.txt`, `trajectory_3d.png`,
`xy.png`, `height.png`, `x_time.png`, `y_time.png`, and a copy of the mission
file. `encoder_angles.csv` contains wall-clock time, elapsed run time, encoder 1
angle, and encoder 2 angle. Encoder recording begins when PX4 confirms Offboard
mode. It stops and durably flushes the CSV when the mission completes, aborts,
or PX4 leaves Offboard. Use **Shutdown / Save Logs** after landing to finalize
the remaining flight artifacts, metadata, plots, and PX4 ULog copy.

Confirm that the completed mission contains angle samples:

```bash
latest_run="$(ls -td "$HOME"/logs/*/ | head -n 1)"
wc -l "$latest_run/encoder_angles.csv"
tail "$latest_run/encoder_angles.csv"
grep 'encoder_angle_rows' "$latest_run/metadata.json"
```

The line count must be greater than one; the first line is the CSV header. The
shutdown helper now waits for the logger to flush and finish its output before
closing tmux. Do not use `tmux kill-session` as the normal shutdown path.

### Download the PX4 `.ulg` without QGroundControl

After manually landing and disarming, keep the flight controller powered and
connected by USB. Download the newest stored PX4 log directly on the Jetson:

```bash
systemctl --user start mavlink-router.service
python3 ~/ROS_PX4/jetson/tools/download_ulog.py
```

The tool uses the router's existing `companion_aux` endpoint on UDP port
`14569`. Leave that port free while downloading; no QGroundControl is required.
If Python reports that `pymavlink` is missing, install it with
`python3 -m pip install --user pymavlink`.

List logs or choose a particular log ID:

```bash
python3 ~/ROS_PX4/jetson/tools/download_ulog.py --list
python3 ~/ROS_PX4/jetson/tools/download_ulog.py --log-id 42
```

By default, downloads go to `~/.local/share/logloader/logs/`. To save directly
into a particular mission run instead, pass its folder explicitly:

```bash
python3 ~/ROS_PX4/jetson/tools/download_ulog.py --output-dir "$HOME/logs/YOUR_RUN_FOLDER"
```

The tool checks for a disarmed PX4 heartbeat, retries missing data, checks the
ULog header, and only renames `.ulg.part` to `.ulg` when the advertised byte
count has been received. Interrupted/failed downloads retain `.part` files;
move or remove that partial file before retrying the same log. Existing output
files are not overwritten. Do not run another log downloader simultaneously.

Mission completion now saves the ROS logs while the vehicle is still hovering;
the complete PX4 flight log must be downloaded after landing and disarming.
A later download does not update the already finalized `metadata.json`.
The flight logger still copies recent `.ulg` files from the logloader folder
when available during finalization.

---

## Troubleshooting

### QGC does not connect

Check which Jetson network mode is active:

```bash
nmcli device status
ip -4 addr show wlP1p1s0
```

If using HomeWifi, the Jetson should have a `192.168.0.x` address.

If using JetsonHotspot, the Jetson should have:

```text
10.42.0.1
```

Find the laptop/QGC IP:

```bash
ip neigh
```

Confirm the QGC router is running:

```bash
systemctl --user status mavlink-router.service
```

Restart the router:

```bash
systemctl --user restart mavlink-router.service
systemctl --user status mavlink-router.service
```

Confirm QGC uses UDP auto-connect on port `14550`.

### HomeWifi does not connect

Check the profile:

```bash
nmcli connection show HomeWifi
```

For hidden Wi-Fi, confirm hidden mode is enabled:

```bash
sudo nmcli connection modify HomeWifi 802-11-wireless.hidden yes
sudo nmcli connection up HomeWifi
```

Check NetworkManager logs:

```bash
journalctl -u NetworkManager -n 80 --no-pager
```

### Hotspot does not start

Check that the Wi-Fi adapter supports AP mode:

```bash
iw list | grep -A 20 "Supported interface modes"
```

Look for:

```text
* AP
```

Restart the hotspot:

```bash
sudo nmcli connection down HomeWifi 2>/dev/null
sudo nmcli connection up JetsonHotspot
```

Check:

```bash
nmcli device status
ip -4 addr show wlP1p1s0
```

Expected hotspot IP:

```text
10.42.0.1
```

### Duplicate JetsonHotspot profiles

List all profiles:

```bash
nmcli connection show
```

If there are duplicate `JetsonHotspot` entries, delete the extra one by UUID:

```bash
sudo nmcli connection delete UUID_TO_DELETE
```

Then confirm:

```bash
nmcli connection show
```

### The launcher reports no DDS topics

Check `dds-agent.service`, the crossed TX/RX wiring and common ground, and run
`uxrce_dds_client status` in the QGC MAVLink Console. Confirm:

```text
UXRCE_DDS_CFG = TELEM 2
SER_TEL2_BAUD = 3000000
```

Also confirm that no other driver is assigned to TELEM2.

Check ROS 2 topics:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep '^/fmu/'
```

If `ros2 topic list` throws a daemon error, reset the ROS 2 daemon:

```bash
ros2 daemon stop
pkill -f ros2-daemon
rm -rf ~/.ros/ros2cli
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep '^/fmu/'
```

### PX4 refuses Offboard mode

Confirm local position is valid and PX4 has no preflight or failsafe warnings.
Use the browser or terminal UI's **Switch Offboard** command; the executor only
starts publishing the Offboard setpoint stream after that request.

### Mission aborts with LOCAL_POSITION_INVALID

This means the mission/safety code does not trust PX4 local position. Do not
fly the mission until local position is valid.

Check:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo --once --qos-reliability best_effort \
  --qos-durability transient_local /fmu/out/vehicle_local_position_v1
```

Look for validity fields such as:

```text
xy_valid: true
z_valid: true
v_xy_valid: true
v_z_valid: true
```

If indoors with no GPS, skip the mission and use QGC motor test with props
removed.

### The mission does not advance after takeoff

The timer intentionally waits at the first setpoint until position error is at
most 0.12 m and speed is at most 0.15 m/s continuously for 1.5 seconds. Check
the executor pane for the reported position error and speed.
