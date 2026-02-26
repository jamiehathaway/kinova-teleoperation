#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="${REPO_ROOT}/.ros2_ws"
SRC_DIR="${WS_ROOT}/src"
PKG_LINK="${SRC_DIR}/kinova_teleoperation"

mkdir -p "${SRC_DIR}"

# Use a relative symlink so it resolves both on host and inside container mount.
EXPECTED_REL_TARGET="../.."

if [ -L "${PKG_LINK}" ]; then
  rm -f "${PKG_LINK}"
elif [ -e "${PKG_LINK}" ]; then
  echo "error: ${PKG_LINK} exists and is not a symlink" >&2
  exit 1
fi

(
  cd "${SRC_DIR}"
  ln -s "${EXPECTED_REL_TARGET}" "$(basename "${PKG_LINK}")"
)

echo "Colcon workspace ready: ${WS_ROOT}"
echo "Package link: ${PKG_LINK} -> ${EXPECTED_REL_TARGET}"
echo "Next commands:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  cd ${WS_ROOT}"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
