#!/bin/bash
# glim_rosbag_docker_outdoor_cpu.sh
# Ubuntu で GLIM ROS2 rosbag を CPU 専用 Docker で実行するスクリプト

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
    echo "  $0 ./rosbags/outdoor_data"
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
if [ ! -d "$HOME/glim_docker/config_L2_outdoor_cpu" ]; then
    echo "エラー: $HOME/glim_docker/config_L2_outdoor_cpu ディレクトリが見つかりません"
    exit 1
fi

if [ ! -d "$HOME/glim_docker/output" ]; then
    echo "output ディレクトリを作成します: $HOME/glim_docker/output"
    mkdir -p "$HOME/glim_docker/output"
fi

# 必要に応じて X11 のアクセス許可
xhost +local:docker 2>/dev/null || echo "X11 アクセス許可の設定をスキップ"

# CPU専用モードで実行
echo "CPU専用モードで実行します（高精度設定）"
GPU_ARGS=""

# Docker コンテナを起動
echo "GLIM Outdoor CPU rosbag処理を開始します..."
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
  -v "$HOME/glim_docker/config_L2_outdoor_cpu":/root/ros2_ws/install/glim/share/glim/config:ro \
  -v "$HOME/glim_docker/output":/tmp/dump \
  -v "$ROSBAG_DIR":/rosbag_data:ro \
  koide3/glim_ros2:humble \
  ros2 run glim_ros glim_rosbag "/rosbag_data/$ROSBAG_NAME"