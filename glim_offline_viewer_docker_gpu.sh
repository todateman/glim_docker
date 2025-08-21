#!/bin/bash
# glim_offline_viewer_docker.sh
# Ubuntu で GLIM Offline Viewer を Docker で起動するスクリプト

set -e

# 必要に応じて X11 のアクセス許可
xhost +local:docker 2>/dev/null || echo "X11 アクセス許可の設定をスキップ"

# GPU使用を有効化
echo "GPU モードで実行します（高速化のため）"
GPU_ARGS="--gpus all"

# TTYが利用可能かチェック
if [ -t 0 ]; then
    DOCKER_TTY_ARGS="-it"
else
    DOCKER_TTY_ARGS=""
fi

docker run \
  $DOCKER_TTY_ARGS \
  --rm \
  --net=host \
  --ipc=host \
  $GPU_ARGS \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/glim_docker/output":/tmp/dump \
  koide3/glim_ros2:humble_cuda12.2 \
  ros2 run glim_ros offline_viewer