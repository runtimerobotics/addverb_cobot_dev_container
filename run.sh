#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="therobocademy/addverb_cobot:latest"

# Use pre-built image if available locally; otherwise pull, then fall back to local build
if docker image inspect "$IMAGE" &>/dev/null; then
    echo "Using local image: $IMAGE"
elif docker pull "$IMAGE" 2>/dev/null; then
    echo "Pulled image: $IMAGE"
else
    echo "Image not available remotely — building locally..."
    docker build -t "$IMAGE" "$SCRIPT_DIR"
fi

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
  "$IMAGE"
