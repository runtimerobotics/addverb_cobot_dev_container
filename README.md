# Cobot ROS 2 Humble Dev Container

Docker-based development environment for the Addverb cobot — ROS 2 Humble on Ubuntu 22.04 with MoveIt 2, Nav2, Gazebo Classic, and `ros2_control`.

## Repository Structure

```
cobot_dev_container/
├── Dockerfile                  # Builds the cobot:deploy image
├── docker-compose.yml          # Compose service: cobot / cobot-humble
├── run.sh                      # Quick docker run wrapper (X11 + GPU)
├── .devcontainer/
│   └── devcontainer.json       # VS Code / Cursor Dev Container config
├── scripts/
│   └── build_workspace.sh      # Rebuild cobot_ros2_ws inside container
├── robot_network_config/
│   └── ip.csv                  # Robot network IP configuration
└── setup_docker_ubuntu.sh      # One-shot Docker install script (Ubuntu)
```

## System Prerequisites

- Ubuntu 22.04 or 24.04 (or Windows 11/10 via WSL2)
- NVIDIA GPU with drivers installed (`nvidia-smi` should work)
- Docker with NVIDIA Container Toolkit

## Docker Installation

### Ubuntu 22.04 / 24.04

```bash
wget https://raw.githubusercontent.com/therobocademy/ros2_nvidia_isaac_bootcamp/refs/heads/main/setup_docker_ubuntu.sh
chmod +x setup_docker_ubuntu.sh
sudo ./setup_docker_ubuntu.sh
```

Or use the script included in this repo:

```bash
sudo ./setup_docker_ubuntu.sh
```

Verify:

```bash
docker --version
docker run --rm --gpus all nvidia/cuda:12.0-base-ubuntu22.04 nvidia-smi
```

### Windows 11/10 (WSL2 + Docker Desktop)

1. Install WSL (Ubuntu 24.04) in PowerShell (as Administrator):

```powershell
wsl --install -d Ubuntu-24.04
```

2. Install Docker Desktop: https://www.docker.com/products/docker-desktop/

3. In Docker Desktop enable: `Settings → Resources → WSL Integration → Ubuntu-24.04`

## Pull the Docker Image

```bash
docker pull therobocademy/addverb_cobot:latest
```

## Running the Container

### Option 1: run.sh (recommended for Linux)

Enables X11 GUI forwarding and GPU access in one command:

```bash
./run.sh
```

This runs the container as user `robot` with:
- Full GPU access (`--gpus all`)
- Host networking
- X11 socket mounted for GUI apps (RViz2, Gazebo, etc.)
- `robot_network_config/` mounted at `/robot_network_config`
- This repo mounted at `/workspaces/cobot_dev`

### Option 2: Docker Compose

```bash
# Linux — allow X11 first
xhost +local:docker
export DISPLAY=${DISPLAY:-:0}

docker compose up -d
```

Attach a shell:

```bash
docker exec -it cobot-humble bash -l
```

Stop:

```bash
docker compose down
```

### Windows (WSL2)

Start an X server on Windows (VcXsrv or X410) with access control disabled, then from the WSL terminal:

```bash
unset DISPLAY   # compose defaults to host.docker.internal:0.0
docker compose up -d
docker exec -it cobot-humble bash -l
```

## Open in Dev Container (VS Code / Cursor)

The `.devcontainer/devcontainer.json` is pre-configured to use `docker-compose.yml`.

### Linux

```bash
xhost +local:docker
export DISPLAY=${DISPLAY:-:0}
```

Then open this folder in VS Code / Cursor and choose **Dev Containers: Reopen in Container**.

Pre-installed VS Code extensions:
- ROS (`ms-iot.vscode-ros`)
- Python, C/C++, CMake, clang-format

### Windows (WSL2)

Start your X server, then reopen the folder in the Dev Container from VS Code.

## Inside the Container

The ROS 2 workspace is at `~/cobot_ros2_ws`. It is sourced automatically in every login shell via `/etc/profile.d/ros2.sh`.

### Rebuild the workspace

```bash
/workspaces/cobot_dev/scripts/build_workspace.sh
```

Or manually:

```bash
source /opt/ros/humble/setup.bash
cd ~/cobot_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Robot network config

Edit `robot_network_config/ip.csv` on the host — it is mounted live into the container at `/robot_network_config/ip.csv`.

## What's Inside the Image

| Component | Version / Package |
|---|---|
| ROS 2 | Humble (desktop-full) |
| MoveIt 2 | `ros-humble-moveit` |
| Nav2 | `ros-humble-navigation2` |
| Gazebo | Classic 11 (`ros-humble-gazebo-ros-pkgs`) |
| ros2_control | full stack |
| UR robot | `ros-humble-ur` |
| User | `robot` (passwordless sudo) |
| Cobot backend | cloned to `/opt/addverb/cobot_backend` |
