#!/usr/bin/env bash
set -euo pipefail

SERVICE="$HOME/.config/systemd/user/mavlink-router.service"
CONFIG_DIR="$HOME/.local/share/mavlink-router"
CONFIG_FILE="$CONFIG_DIR/main.conf"
TEMPLATE="$HOME/ROS_PX4/config/mavlink-router-main.conf"

echo
echo "Available nearby IPs:"
ip neigh | awk '{print "  " $1 "  " $3 "  " $5}' || true
echo
echo "Current Jetson IPs:"
ip -4 addr show | awk '/inet / {print "  " $2}'
echo

mkdir -p "$HOME/.config/systemd/user"
mkdir -p "$CONFIG_DIR"
install -m 0644 "$HOME/ROS_PX4/config/mavlink-router.service" "$SERVICE"
install -m 0644 "$TEMPLATE" "$CONFIG_FILE"

systemctl --user daemon-reload
systemctl --user enable --now mavlink-router.service
systemctl --user restart mavlink-router.service

echo
echo "ARK MAVLink router is listening for QGroundControl on UDP port 14550."
echo "Connect QGC to the Jetson IP on your current network, for example:"
echo "  Hotspot: 10.42.0.1:14550"
echo
systemctl --user --no-pager status mavlink-router.service
