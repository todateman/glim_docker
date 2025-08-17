#!/bin/bash
# glim_rosnode_docker.sh
# Ubuntu で GLIM ROS2 ノードを Docker で起動するスクリプト

set -e

# 必要なディレクトリが存在することを確認
if [ ! -d "$HOME/glim_docker/config_L2_defalt" ]; then
    echo "エラー: $HOME/glim_docker/config_L2_defalt ディレクトリが見つかりません"
    exit 1
fi

if [ ! -d "$HOME/glim_docker/output" ]; then
    echo "output ディレクトリを作成します: $HOME/glim_docker/output"
    mkdir -p "$HOME/glim_docker/output"
fi

# 必要に応じて X11 のアクセス許可
xhost +local:docker 2>/dev/null || echo "X11 アクセス許可の設定をスキップ"

# GPU使用を無効にしてCPU版を使用
echo "CPU モードで実行します（GPU版でメモリエラーが発生するため）"
GPU_ARGS=""

# Docker コンテナを起動
echo "GLIM ROS2 ノードを起動します..."
docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --pid=host \
  $GPU_ARGS \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/glim_docker/config_L2_defalt":/root/ros2_ws/install/glim/share/glim/config:ro \
  -v "$HOME/glim_docker/output":/tmp/dump \
  koide3/glim_ros2:humble \
  ros2 run glim_ros glim_rosnode