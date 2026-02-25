#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="kinova-ros2-jazzy:dev"
CONTAINER_NAME="kinova-ros2-jazzy-dev"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<USAGE
Usage: $0 <build|run|exec|stop|rm|test>

Commands:
  build   Build the local ROS 2 Jazzy image
  run     Start (or restart) a dev container with project mounted
  exec    Open interactive shell in running container
  stop    Stop running container
  rm      Remove container
  test    Verify ROS 2 works (prints distro and topic list)
USAGE
}

build_image() {
  podman build -t "${IMAGE_NAME}" -f "${PROJECT_ROOT}/container/Containerfile" "${PROJECT_ROOT}"
}

run_container() {
  if podman container exists "${CONTAINER_NAME}"; then
    podman rm -f "${CONTAINER_NAME}" >/dev/null
  fi

  podman run -d \
    --name "${CONTAINER_NAME}" \
    --hostname kinova-ros2 \
    --network host \
    --ipc host \
    -v "${PROJECT_ROOT}:/workspaces/kinova-teleoperation:Z" \
    -w /workspaces/kinova-teleoperation \
    "${IMAGE_NAME}" \
    sleep infinity
}

exec_container() {
  podman exec -it "${CONTAINER_NAME}" bash -lc "source /opt/ros/jazzy/setup.bash && exec bash"
}

stop_container() {
  podman stop "${CONTAINER_NAME}"
}

rm_container() {
  podman rm -f "${CONTAINER_NAME}"
}

test_ros2() {
  podman exec "${CONTAINER_NAME}" bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 doctor --report; ros2 topic list'
}

[[ $# -eq 1 ]] || { usage; exit 1; }

case "$1" in
  build) build_image ;;
  run) run_container ;;
  exec) exec_container ;;
  stop) stop_container ;;
  rm) rm_container ;;
  test) test_ros2 ;;
  *) usage; exit 1 ;;
esac
