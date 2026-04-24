#!/bin/bash
# Start PX4+Gazebo with Gobi v2 (QuadraticDrag). Run from repo root.
set -e
REPO="${GOBIREPO:-$HOME/github_code/gazebo-px4-sim}"
PLUGIN="${REPO}/plugins/quadratic_drag/build"
export GZ_SIM_RESOURCE_PATH="${REPO}/PX4-Autopilot/Tools/simulation/gz/models:${REPO}/PX4-Autopilot/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PLUGIN}:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export PX4_SYS_AUTOSTART=4097
export PX4_GZ_MODEL=gobi_v2
export HEADLESS=1
ROOTFS="${REPO}/PX4-Autopilot/build/px4_sitl_default/rootfs"
ETC="${REPO}/PX4-Autopilot/build/px4_sitl_default/etc"
BIN="${REPO}/PX4-Autopilot/build/px4_sitl_default/bin/px4"
if [[ ! -x "$BIN" ]]; then
  echo "Build px4_sitl first. Expected: $BIN" >&2
  exit 1
fi
if [[ ! -f "${PLUGIN}/libQuadraticDrag.so" ]]; then
  echo "Build QuadraticDrag: cd $REPO/plugins/quadratic_drag/build && cmake .. && make" >&2
  exit 1
fi
cd "$ROOTFS"
rm -f parameters.bson parameters_backup.bson dataman 2>/dev/null || true
exec "$BIN" "$ETC"
