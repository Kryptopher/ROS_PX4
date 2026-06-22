# Real Drone Mission Guide

This guide runs a TSV mission on the physical PX4 vehicle using ROS 2 on the
Jetson. It does not start PX4 SITL or Gazebo.

> **Flight safety:** Remove the propellers for setup and bench testing. For the
> first powered flight, use the short `hover_1m_test.tsv` mission in a clear
> outdoor area with a pilot holding the RC transmitter. Confirm that the RC
> mode switch, Land, RTL, and emergency stop behavior work before using
> Offboard mode. Follow local operating rules and the vehicle's normal
> preflight checklist.

---

## How the hardware is connected

* Holybro Pixhawk 6X TELEM3 carries uXRCE-DDS data to the Jetson for ROS 2.
* The Pixhawk USB-C connection carries MAVLink to the Jetson.
* `mavlink-qgc-bridge.service` forwards USB MAVLink from the Pixhawk to
  QGroundControl over UDP port `14550`.
* The mission launcher runs the executor, safety monitor, flight logger, and
  optional payload encoder on the Jetson.

This vehicle uses a Holybro Pixhawk 6X with the PX4 FMUv6X architecture. On the
Pixhawk, TELEM3 is `/dev/ttyS1`. The cable connects Pixhawk TELEM3 TX to Jetson
RX, Pixhawk TELEM3 RX to Jetson TX, and ground to ground. On this Jetson, the
selected UART is `/dev/ttyTHS1`.

Leave the Pixhawk USB-C cable connected. USB-C remains dedicated to MAVLink and
the QGC bridge, while TELEM3 is dedicated to uXRCE-DDS.

Recommended link split:

```text
Pixhawk USB-C  -> Jetson USB       -> MAVLink / QGroundControl bridge
Pixhawk TELEM3 -> Jetson UART      -> uXRCE-DDS / ROS 2 Offboard
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
cd ~/ROS_PX4
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The launcher sources the existing `~/px4_ros2_ws/install/setup.bash`. Install
the package into that workspace:

```bash
mkdir -p ~/px4_ros2_ws/src
ln -sfn ~/ROS_PX4/zed_px4_bridge_folder ~/px4_ros2_ws/src/zed_px4_bridge
cd ~/px4_ros2_ws
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
within 30 seconds, it switches to `JetsonHotspot`.

Create the script:

```bash
sudo tee /usr/local/bin/drone-network-autoswitch.sh >/dev/null <<'SH'
#!/usr/bin/env bash
set -u

WIFI_IF="wlP1p1s0"
HOME_CON="HomeWifi"
HOTSPOT_CON="JetsonHotspot"
WAIT_SECONDS=30

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

## QGroundControl MAVLink IP target

The Pixhawk USB-C cable carries MAVLink to the Jetson. The Jetson forwards that
MAVLink stream to QGroundControl over UDP port `14550`.

Because different people may use different laptops, do not permanently hardcode
one QGC laptop IP. Instead, use the interactive QGC target script below.

### Create the interactive QGC target script

```bash
mkdir -p ~/ROS_PX4/tools

cat > ~/ROS_PX4/tools/set_qgc_target.sh <<'SH'
#!/usr/bin/env bash
set -euo pipefail

SERVICE="$HOME/.config/systemd/user/mavlink-qgc-bridge.service"
SERIAL="/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00"
BRIDGE="/home/scs/ROS_PX4/tools/mavlink_udp_bridge.py"
PORT="14550"

echo
echo "Available nearby IPs:"
ip neigh | awk '{print "  " $1 "  " $3 "  " $5}' || true
echo
echo "Current Jetson IPs:"
ip -4 addr show | awk '/inet / {print "  " $2}'
echo

read -rp "Enter QGroundControl laptop IP: " TARGET

if [[ ! "$TARGET" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "Error: '$TARGET' does not look like an IPv4 address."
  exit 1
fi

mkdir -p "$HOME/.config/systemd/user"

cat > "$SERVICE" <<EOF
[Unit]
Description=Pixhawk USB to QGroundControl UDP bridge
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 $BRIDGE --serial $SERIAL --target $TARGET --port $PORT
Restart=always
RestartSec=2

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user restart mavlink-qgc-bridge.service

echo
echo "MAVLink bridge target set to:"
echo "  $TARGET:$PORT"
echo
systemctl --user --no-pager status mavlink-qgc-bridge.service
SH

chmod +x ~/ROS_PX4/tools/set_qgc_target.sh
```

Add an easy command:

```bash
echo "alias drone_qgc_ip='~/ROS_PX4/tools/set_qgc_target.sh'" >> ~/.bashrc
source ~/.bashrc
```

### Use the interactive QGC target command

Run:

```bash
drone_qgc_ip
```

Then enter the laptop's QGC IP.

For HomeWifi mode, the laptop IP is usually:

```text
192.168.0.xxx
```

For JetsonHotspot mode, the laptop IP is usually:

```text
10.42.0.xxx
```

To find connected hotspot clients from the Jetson:

```bash
ip neigh
```

Example output:

```text
10.42.0.23 dev wlP1p1s0 lladdr xx:xx:xx:xx:xx:xx REACHABLE
```

Then run:

```bash
drone_qgc_ip
```

and enter:

```text
10.42.0.23
```

QGroundControl should use UDP auto-connect on port `14550`. Do not create a
second manual QGC link on the same port.

---

## QGC bridge check

Confirm the bridge is running:

```bash
systemctl --user status mavlink-qgc-bridge.service
```

Show the current target:

```bash
systemctl --user cat mavlink-qgc-bridge.service
```

Restart the bridge:

```bash
systemctl --user restart mavlink-qgc-bridge.service
```

---

## Jetson R36.5 UART fix

This Jetson's L4T R36.5 device tree shipped with broken DMA properties for
UART1, causing null bytes and truncated serial data. The fix is already
installed on this vehicle and forces `/dev/ttyTHS1` to reliable PIO mode. If
the boot files are replaced by an OS update and the UART loopback regresses,
reapply it with:

```bash
sudo ~/ROS_PX4/tools/install_jetson_r36_5_uart_fix.sh
sudo reboot
```

---

## DDS agent setup

Install and enable the Holybro Pixhawk TELEM3 DDS agent service:

```bash
mkdir -p ~/.config/systemd/user
install -m 0644 ~/ROS_PX4/config/dds-agent.service \
  ~/.config/systemd/user/dds-agent.service
systemctl --user daemon-reload
systemctl --user enable --now dds-agent.service
systemctl --user status dds-agent.service
```

This service runs `MicroXRCEAgent` on the Jetson's `/dev/ttyTHS1` UART at
3,000,000 baud. Confirm the physical TELEM3 cable is connected to that UART
before expecting DDS topics.

Configure the matching PX4 side once in QGC Parameters:

```text
UXRCE_DDS_CFG = TELEM 3
SER_TEL3_BAUD = 3000000
```

Some newer PX4 builds expose `UXRCE_DDS_FLCTRL`. It is not present on this
vehicle's firmware and is not required for the three-wire TX/RX/ground link.

Ensure no `MAV_*_CONFIG` or other serial driver is assigned to TELEM3. Reboot
PX4 with the propellers removed. In the QGC MAVLink Console, verify:

```text
uxrce_dds_client status
```

It should report `Running, connected`. QGC continues to use the separate USB-C
MAVLink bridge.

If the relayed QGC MAVLink Console is blank, verify DDS directly on the Jetson:

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo --once --qos-reliability best_effort \
  --qos-durability transient_local /fmu/out/sensor_combined
```

A live sample confirms the PX4 client, TELEM3 link, Jetson agent, and ROS 2 DDS
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
source ~/px4_ros2_ws/install/setup.bash
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
cd ~/ROS_PX4
START_ENCODER=false scripts/run_dds_mission.sh missions/hover_1m_test.tsv
```

The executor should say that it is waiting for PX4 to be armed and in Offboard.
It must continuously publish setpoints before PX4 will accept Offboard mode.

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

The recommended first flight is `missions/hover_1m_test.tsv`:

* Rise to 1 m above the local origin.
* Hold that position for 20 seconds.
* Continue holding at mission completion until the pilot lands.

Mission coordinates are relative to PX4's local origin:

* `x`: north/forward
* `y`: east/right
* `z`: positive upward
* `heading_deg`: compass heading clockwise from north

The mission file is tab-delimited. Review it before every flight:

```bash
column -s $'\t' -t missions/hover_1m_test.tsv
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

Connect the QGC laptop to the correct network, then set the QGC target IP:

```bash
drone_qgc_ip
```

Confirm QGC shows live attitude, battery, GPS, and arming status.

Start the conservative hover mission:

```bash
cd ~/ROS_PX4
RUN_LABEL=first_hover \
START_ENCODER=false \
SAFETY_MAX_ALTITUDE_M=2.0 \
SAFETY_WARN_ALTITUDE_M=1.5 \
SAFETY_MAX_VELOCITY_MS=1.5 \
SAFETY_LOCAL_RADIUS_M=5.0 \
scripts/run_dds_mission.sh missions/hover_1m_test.tsv
```

Use `START_ENCODER=true` only when the payload encoder hardware is connected.

Wait until the executor reports that it has loaded the mission, has local
position, and is waiting for arming and Offboard mode. Check QGC once more for
warnings. Then, when the pilot is ready:

1. Select **Offboard** in QGC or with the assigned RC mode switch.
2. Arm the vehicle.
3. Keep hands on the controls and watch the flight, QGC status, and tmux panes.

Automatic arming and automatic Offboard selection are deliberately disabled.
The mission clock starts only after PX4 is armed, in Offboard, and settled at
the first setpoint for 1.5 seconds.

---

## 4. End or abort the flight

At the `end` row, the executor keeps streaming the final setpoint. It does not
land or disarm the real vehicle automatically.

Land using QGC or the RC transmitter. Leave the mission processes running until
the vehicle is on the ground and disarmed.

If anything looks wrong, the pilot should immediately use the safest available
recovery action for the situation:

* Switch out of Offboard into a tested manual/position mode to take control.
* Select Land when landing in place is safest.
* Select RTL only when the home position, altitude, and return path are safe.
* Use the emergency stop/kill function only for an actual emergency and only
  with full awareness that thrust will stop immediately.

Do not stop the executor while the vehicle is flying. Loss of its setpoint
stream makes PX4 invoke the configured Offboard-loss failsafe.

---

## 5. Shut down and collect logs

After landing and disarming, stop each pane cleanly so the logger closes its
files:

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

Each run contains `flight.csv`, `events.csv`, `metadata.json`, and a copy of the
mission file. Also download the PX4 `.ulg` log from QGC before changing the
vehicle or mission configuration.

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

Set the QGC target:

```bash
drone_qgc_ip
```

Restart the bridge:

```bash
systemctl --user restart mavlink-qgc-bridge.service
systemctl --user status mavlink-qgc-bridge.service
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
UXRCE_DDS_CFG = TELEM 3
SER_TEL3_BAUD = 3000000
```

Also confirm that no other driver is assigned to TELEM3.

Check ROS 2 topics:

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic list | grep '^/fmu/'
```

If `ros2 topic list` throws a daemon error, reset the ROS 2 daemon:

```bash
ros2 daemon stop
pkill -f ros2-daemon
rm -rf ~/.ros/ros2cli
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic list | grep '^/fmu/'
```

### PX4 refuses Offboard mode

Confirm that the executor pane is still publishing, local position is valid,
and PX4 has no preflight or failsafe warnings. Start the launcher before asking
PX4 to enter Offboard.

### Mission aborts with LOCAL_POSITION_INVALID

This means the mission/safety code does not trust PX4 local position. Do not
fly the mission until local position is valid.

Check:

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash
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
