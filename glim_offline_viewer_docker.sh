#!/bin/bash
# glim_offline_viewer_docker.sh
# Ubuntu で GLIM Offline Viewer を Docker で起動するスクリプト

set -e

# 必要に応じて X11 のアクセス許可
xhost +local:docker 2>/dev/null || echo "X11 アクセス許可の設定をスキップ"

# CPU版を使用
echo "CPU モードで実行します"

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
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/glim_docker/output":/tmp/dump \
  koide3/glim_ros2:humble \
  ros2 run glim_ros offline_viewer