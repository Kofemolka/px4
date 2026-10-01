#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4="$(cd "${SCRIPT_DIR}/../../Tools/simulation/gz" && pwd)"

export GZ_SIM_RESOURCE_PATH=$PX4/models:$PX4/worlds
export GZ_SIM_SERVER_CONFIG_PATH=$SCRIPT_DIR/server.config

# Gazebo GUI (Qt + ogre2) renders a black/frozen 3D view under native Wayland
export QT_QPA_PLATFORM=xcb

gz sim -r -s "$SCRIPT_DIR/world.sdf" &
server_pid=$!
trap 'kill $server_pid 2>/dev/null' EXIT

if [[ "$1" != "--headless" ]]; then
	gz sim -g
else
	wait $server_pid
fi
