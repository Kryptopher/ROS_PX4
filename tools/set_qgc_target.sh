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
