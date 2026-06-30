#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_ROOT="${UAV_USV_BUILD_ROOT:-/var/tmp/UAV_USV_team_build}"
INSTALL_ROOT="${UAV_USV_INSTALL_ROOT:-/var/tmp/UAV_USV_team_install}"
LOG_ROOT="${UAV_USV_LOG_ROOT:-/var/tmp/UAV_USV_team_log}"

set +u
source /opt/ros/humble/setup.bash
set -u

find "${WORKSPACE_ROOT}/src" -type f -name '*.py' -print0 \
  | xargs -0 -r python3 -m py_compile

find "${WORKSPACE_ROOT}/src" -type f \
  \( -name 'package.xml' -o -name '*.sdf' -o -name '*.urdf' -o -name '*.xacro' \) \
  -print0 | xargs -0 -r xmllint --noout

cd "${WORKSPACE_ROOT}"
colcon --log-base "${LOG_ROOT}" build \
  --build-base "${BUILD_ROOT}" \
  --install-base "${INSTALL_ROOT}" \
  --symlink-install \
  --event-handlers console_direct+

set +u
source "${INSTALL_ROOT}/setup.bash"
set -u
colcon --log-base "${LOG_ROOT}" test \
  --build-base "${BUILD_ROOT}" \
  --install-base "${INSTALL_ROOT}" \
  --event-handlers console_direct+

colcon --log-base "${LOG_ROOT}" test-result --verbose

echo "Workspace checks passed."
