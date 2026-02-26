# ROS 2 Jazzy with Podman

This project uses a local Podman image so ROS 2 Jazzy is not installed on the host.

## Prerequisites
- Podman installed on host
- Run from repo root (`kinova-teleoperation`)

## Quick start
```bash
./scripts/podman_ros2.sh build
./scripts/podman_ros2.sh run
./scripts/podman_ros2.sh exec
```

Then inside the container shell:
```bash
source /opt/ros/jazzy/setup.bash
./scripts/setup_colcon_ws.sh
cd .ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Recommended workspace layout
This repo stays as a normal git repository root. A local colcon workspace is created in:
- `.ros2_ws/`

The bootstrap script creates this symlink:
- `.ros2_ws/src/kinova_teleoperation -> <repo-root>`

This gives you a reproducible workflow while keeping git history and container config at repository root.

## Commands
- `build`: build image `kinova-ros2-jazzy:dev`
- `run`: start container `kinova-ros2-jazzy-dev` with project bind-mounted
- `exec`: open shell with ROS sourced
- `test`: run ROS smoke test (`ros2 doctor`, `ros2 topic list`)
- `stop`: stop container
- `rm`: remove container

## VS Code Dev Containers
A `.devcontainer/devcontainer.json` is included and points to `container/Containerfile`.
If using Podman with VS Code Dev Containers, ensure your VS Code environment is configured to use Podman as the container runtime.

## Notes
- Host network mode is enabled for ROS discovery.
- No robot is required for smoke tests.