#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

xhost +local:docker

docker run \
  --name cobot-humble \
  --entrypoint bash \
  -it \
  --rm \
  --gpus all \
  --network host \
  --privileged \
  -e DISPLAY \
  -e XAUTHORITY=/home/robot/.Xauthority \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT=0 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -v "$HOME/.Xauthority:/home/robot/.Xauthority" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v "${SCRIPT_DIR}/robot_network_config:/robot_network_config" \
  -v "${SCRIPT_DIR}:/workspaces/cobot_dev:cached" \
  cobot:deploy
