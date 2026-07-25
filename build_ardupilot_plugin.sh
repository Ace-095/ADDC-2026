#!/bin/bash
# build_ardupilot_plugin.sh
# Clones and builds the ardupilot_gazebo plugin from source.
# Run this ONCE from any terminal. Only needs internet access.
set -e

PLUGIN_BUILD_DIR="${HOME}/gz_ws/src/ardupilot_gazebo_full"

echo "=== Building ArduPilotPlugin for Gazebo Harmonic ==="

# Clone the repo
if [ ! -d "${PLUGIN_BUILD_DIR}" ]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git "${PLUGIN_BUILD_DIR}"
else
    echo "Source already exists at ${PLUGIN_BUILD_DIR}, updating..."
    git -C "${PLUGIN_BUILD_DIR}" pull
fi

# Build
mkdir -p "${PLUGIN_BUILD_DIR}/build"
cd "${PLUGIN_BUILD_DIR}/build"

cmake .. \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo

make -j$(nproc)

echo ""
echo "=== Build successful! ==="
echo "Plugin installed at: ${PLUGIN_BUILD_DIR}/build/lib/libArduPilotPlugin.so"
echo ""
echo "Now update launch_sim.sh plugin path:"
echo "  export GZ_SIM_SYSTEM_PLUGIN_PATH=\"${PLUGIN_BUILD_DIR}/build/lib:\${GZ_SIM_SYSTEM_PLUGIN_PATH}\""
echo ""
echo "Or add this line permanently to your ~/.bashrc:"
echo "  export GZ_SIM_SYSTEM_PLUGIN_PATH=\"${PLUGIN_BUILD_DIR}/build/lib:\${GZ_SIM_SYSTEM_PLUGIN_PATH}\""
