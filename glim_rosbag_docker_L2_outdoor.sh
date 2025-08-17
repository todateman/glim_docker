#!/bin/bash
# glim_rosbag_docker.sh
# Ubuntu で GLIM ROS2 rosbag を Docker で実行するスクリプト

set -e

# 使用方法を表示する関数
show_usage() {
    echo "使用方法: $0 <rosbag_path> [options]"
    echo ""
    echo "引数:"
    echo "  rosbag_path    処理するrosbagファイルのパス"
    echo ""
    echo "オプション:"
    echo "  -h, --help     このヘルプメッセージを表示"
    echo ""
    echo "例:"
    echo "  $0 /path/to/data.bag"
    echo "  $0 ./rosbags/os1_128_01"
}

# 引数をチェック
if [ $# -eq 0 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    show_usage
    exit 0
fi

ROSBAG_PATH="$1"

# rosbagファイルの存在確認
if [ ! -e "$ROSBAG_PATH" ]; then
    echo "エラー: rosbagファイルが見つかりません: $ROSBAG_PATH"
    exit 1
fi

# 絶対パスに変換
ROSBAG_PATH=$(realpath "$ROSBAG_PATH")
ROSBAG_DIR=$(dirname "$ROSBAG_PATH")
ROSBAG_NAME=$(basename "$ROSBAG_PATH")

echo "処理するrosbag: $ROSBAG_PATH"

# 必要なディレクトリが存在することを確認
if [ ! -d "$HOME/glim_docker/config_L2_outdoor_gpu" ]; then
    echo "エラー: $HOME/glim_docker/config_L2_outdoor_gpu ディレクトリが見つかりません"
    exit 1
fi

if [ ! -d "$HOME/glim_docker/output" ]; then
    echo "output ディレクトリを作成します: $HOME/glim_docker/output"
    mkdir -p "$HOME/glim_docker/output"
fi

# 必要に応じて X11 のアクセス許可
xhost +local:docker 2>/dev/null || echo "X11 アクセス許可の設定をスキップ"

# GPU使用を有効化
echo "GPU モードで実行します（高速化のため）"
GPU_ARGS="--gpus all"

# Docker コンテナを起動
echo "GLIM rosbag処理を開始します..."
echo "rosbag: $ROSBAG_NAME"
echo "出力ディレクトリ: $HOME/glim_docker/output"

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
  --pid=host \
  $GPU_ARGS \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/glim_docker/config_L2_outdoor_gpu":/root/ros2_ws/install/glim/share/glim/config:ro \
  -v "$HOME/glim_docker/output":/tmp/dump \
  -v "$ROSBAG_DIR":/rosbag_data:ro \
  koide3/glim_ros2:humble_cuda12.2 \
  ros2 run glim_ros glim_rosbag "/rosbag_data/$ROSBAG_NAME"