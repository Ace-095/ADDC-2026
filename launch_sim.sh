#!/bin/bash
# Gazebo SITL Launcher - fixes trailing colon bug in GZ_SIM_RESOURCE_PATH
set -e  # exit on error so we see what fails

DRONE_PI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Ensure ~/.gz exists (gz-common segfaults if it can't create this)
mkdir -p "${HOME}/.gz/sim/8"
mkdir -p "${HOME}/.gz/fuel"

# 2. Fix GZ_SIM_RESOURCE_PATH: strip trailing colon(s), prepend local models
BASE_RESOURCE="${GZ_SIM_RESOURCE_PATH%:}"
export GZ_SIM_RESOURCE_PATH="${DRONE_PI}/models:${BASE_RESOURCE}"

# Keep existing system plugin path, prepend newly compiled ardupilot_gazebo build
_AP_BUILD="/home/ace/gz_ws/src/ardupilot_gazebo_full/build"
if [ -f "${_AP_BUILD}/lib/libArduPilotPlugin.so" ]; then
    _AP_PLUGIN_PATH="${_AP_BUILD}/lib"
else
    _AP_PLUGIN_PATH="${_AP_BUILD}"
fi
export GZ_SIM_SYSTEM_PLUGIN_PATH="${_AP_PLUGIN_PATH}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

# 4. Ensure no wayland issues
export QT_QPA_PLATFORM=xcb

echo "========================================"
echo "[launch_sim] HOME                = ${HOME}"
echo "[launch_sim] GZ_SIM_RESOURCE_PATH = ${GZ_SIM_RESOURCE_PATH}"
echo "[launch_sim] GZ_SIM_SYSTEM_PLUGIN_PATH = ${GZ_SIM_SYSTEM_PLUGIN_PATH}"
echo "[launch_sim] ~/.gz exists        = $(ls -d ~/.gz 2>/dev/null && echo YES || echo NO)"
echo "========================================"

exec gz sim -v4 -r "${DRONE_PI}/iris_runway.sdf"
