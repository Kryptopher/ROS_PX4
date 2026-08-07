#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLATFORM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROOT_DIR="$(cd "$PLATFORM_DIR/.." && pwd)"

export ROS_PX4_PLATFORM="${ROS_PX4_PLATFORM:-pi}"
export ROS_PX4_PLATFORM_HOME="${ROS_PX4_PLATFORM_HOME:-$PLATFORM_DIR}"
export ROS_PX4_SHARED_HOME="${ROS_PX4_SHARED_HOME:-$ROOT_DIR/shared}"

exec "$ROS_PX4_SHARED_HOME/scripts/kill_sitl.sh" "$@"
