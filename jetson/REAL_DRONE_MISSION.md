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
ln -sfn ~/ROS_PX4/jetson/zed_px4_bridge_folder ~/ros2_ws/src/zed_px4_bridge
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
START_ENCODER=false scripts/run_dds_mission.sh missions/Default.tsv
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

Start the selected flight mission. Set `MISSION_FILE` to the TSV you intend to
fly and choose safety limits that are above the planned mission envelope but
still conservative for the test site:

```bash
cd ~/ROS_PX4/jetson
MISSION_FILE=$HOME/ROS_PX4/jetson/missions/Default.tsv \
START_ENCODER=false \
MISSION_DIR=$HOME/ROS_PX4/jetson/missions \
SAFETY_MAX_ALTITUDE_M=50.0 \
SAFETY_WARN_ALTITUDE_M=48.0 \
SAFETY_MAX_VELOCITY_MS=7.0 \
SAFETY_LOCAL_RADIUS_M=50.0 \
scripts/run_dds_mission.sh "$MISSION_FILE"
```

Use `START_ENCODER=true` only when the payload encoder hardware is connected.
The terminal UI's **Select Mission** option lists `.tsv` files from
`MISSION_DIR`, which defaults to `~/ROS_PX4/jetson/missions`. If you select a different
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

At the `end` row, the executor keeps streaming the final setpoint only while PX4
remains in Offboard. It does not land or disarm the real vehicle automatically.

Land using QGC or the RC transmitter. If PX4 leaves Offboard because the pilot
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

After landing and disarming, select **Shutdown / Save Logs** in the browser or
terminal mission-control UI. This stops each mission pane cleanly so the logger
closes its files, then closes the `px4_dds_mission` tmux session.

If the UI is not available, run the same shutdown helper from another terminal:

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

Each run contains `flight.csv`, `events.csv`, `commands.csv`, `metadata.json`,
`summary.txt`, `trajectory_3d.png`, `xy.png`, `height.png`, `x_time.png`,
`y_time.png`, and a copy of the mission file. The logger also tries to copy the
newest PX4 `.ulg` from `~/.local/share/logloader/logs` into the same run folder
when the mission is shut down. If `metadata.json` says `"ulog_file": ""`,
logloader had not downloaded the PX4 log yet; download it from QGC/logloader and
copy it into the run folder manually.

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
