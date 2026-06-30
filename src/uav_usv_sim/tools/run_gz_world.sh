#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_PREFIX="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SOURCE_PREFIX="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [ -d "${INSTALL_PREFIX}/share/uav_usv_sim" ]; then
  SHARE_DIR="${INSTALL_PREFIX}/share/uav_usv_sim"
  PLUGIN_DIR="${INSTALL_PREFIX}/lib/uav_usv_sim/plugins"
else
  SHARE_DIR="${SOURCE_PREFIX}"
  PLUGIN_DIR="${SHARE_DIR}/../../build/uav_usv_sim"
fi

export GZ_SIM_RESOURCE_PATH="${SHARE_DIR}/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PLUGIN_DIR}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

gz sim -r "${SHARE_DIR}/worlds/default.sdf"
