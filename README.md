# GLIM Docker

このプロジェクトは、[GLIM](https://koide3.github.io/glim/) ([GitHub: koide3/glim_ros2](https://github.com/koide3/glim_ros2)) を使用してLiDAR SLAMによる点群地図作成をDockerコンテナ環境で実行します。

## 環境

### 対応LiDARセンサー

#### Unitree D4 LiDAR L2
- [Unitree D4 LiDAR L2](https://www.unitree.com/L2)
  - SDK: https://github.com/unitreerobotics/unilidar_sdk2
  - ROS2 トピック
    - LiDAR: `/unilidar/cloud`
    - IMU: `/unilidar/imu`
  - LiDAR-IMU変換: `[0.007698, 0.014655, -0.00667, 0.0, 0.0, 0.0, 1.0]`
  - スキャン距離: 30m / 90%
  - 近距離ブラインドエリア: 0.05m
  - 有効周波数: 64000points/s
  - 周回スキャン周波数: 5.55Hz
  - IMU: 3軸加速度計 + 3軸ジャイロスコープ

#### Livox Mid-360
- [Livox Mid-360](https://www.livoxtech.com/mid-360)
  - ROS2 トピック
    - LiDAR: `/livox/lidar`
    - IMU: `/livox/imu`
  - LiDAR-IMU変換: `[-0.011, -0.02329, 0.04412, 0.0, 0.0, 0.0, 1.0]`
  - 検出範囲: 70m @ 80% 反射率
  - 視野角: 360° × 59°
  - スキャンレート: 10Hz
  - IMU: 6軸（3軸加速度計 + 3軸ジャイロスコープ）

### PC環境
- GPU: NVIDIA GeForce RTX 5060Ti

## 環境構築（参考：https://koide3.github.io/glim/docker.html）

### GPUで処理の場合

```bash
# 設定ファイルをコピーして編集
git clone https://github.com/koide3/glim.git /tmp/glim
cp -R /tmp/glim/config $HOME/glim_docker/config

# Docker Hubからイメージを取得
docker pull koide3/glim_ros2:humble_cuda12.2
```

以下は単体でSLAMを実行する場合のコマンド。  
[後述のシェルスクリプト](#使用方法)を使用する方法が簡単なのでおすすめ。

```bash
# GPU およびDISPLAY サポート付きでglim_ros2:humble_cuda12.2 イメージを起動
# 別ターミナルでrosbagファイルを再生すること
docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --pid=host \
  --gpus all \
  -e=DISPLAY \
  -e=ROS_DOMAIN_ID \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/glim_docker/config:/glim/config \
  koide3/glim_ros2:humble_cuda12.2 \
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config
```

### CPUで処理（GPUなし）の場合

```bash
# 設定ファイルをコピーして編集
git clone https://github.com/koide3/glim.git /tmp/glim
cp -R /tmp/glim/config $HOME/glim_docker/config

# 以下のように変更:
# "config_odometry" : "config_odometry_cpu.json"
# "config_sub_mapping" : "config_sub_mapping_cpu.json"
# "config_global_mapping" : "config_global_mapping_cpu.json"
nano config/config.json

# Docker Hubからイメージを取得
docker pull koide3/glim_ros2:humble
```

以下は単体でSLAMを実行する場合のコマンド。  
[後述のシェルスクリプト](#使用方法)を使用する方法が簡単なのでおすすめ。

```bash
# DISPLAYサポート付きでglim_ros2:humble イメージを起動
# 別ターミナルでrosbagファイルを再生すること
docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --pid=host \
  -e=DISPLAY \
  -e=ROS_DOMAIN_ID \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/glim_docker/config:/glim/config \
  koide3/glim_ros2:humble \
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config
```

### （参考）ソースからDockerイメージをビルド

必須ではないが、Docker Hubからイメージを取得しない場合は下記のコマンドでDockerイメージをビルドする。

```bash
mkdir /tmp/glim_docker && cd /tmp/glim_docker
git clone https://github.com/koide3/glim.git 
git clone https://github.com/koide3/glim_ros2.git

# GPU
docker build \
  -f glim_ros2/docker/Dockerfile.gcc \
  --build-arg="BASE_IMAGE=koide3/gtsam_points:jammy" \
  --build-arg="ROS_DISTRO=humble" \
  --tag glim_ros2:humble \
  .

# CPU
docker build \
  -f glim_ros2/docker/Dockerfile.gcc.cuda \
  --build-arg="BASE_IMAGE=koide3/gtsam_points:jammy_cuda12.2" \
  --build-arg="ROS_DISTRO=humble" \
  --tag glim_ros2:humble_cuda12.2 \
  .
```

## ディレクトリ構造

```
$HOME
└── glim_docker/
        ├── config/                                // デフォルト設定（オリジナルGLIM）
        ├── config_L2_default/                     // Unitree L2 デフォルト設定
        ├── config_L2_indoor_gpu/                  // Unitree L2 屋内GPU設定
        ├── config_L2_indoor_cpu/                  // Unitree L2 屋内CPU設定
        ├── config_L2_outdoor_gpu/                 // Unitree L2 屋外GPU設定
        ├── config_L2_outdoor_cpu/                 // Unitree L2 屋外CPU設定
        ├── config_mid360/                         // Livox Mid-360 設定
        ├── glim_offline_viewer_docker_cpu.sh      // オフラインビューア（CPU）
        ├── glim_offline_viewer_docker_gpu.sh      // オフラインビューア（GPU）
        ├── glim_rosbag_docker_L2_indoor_gpu.sh    // Unitree L2 屋内GPU rosbagマッピング
        ├── glim_rosbag_docker_L2_indoor_cpu.sh    // Unitree L2 屋内CPU rosbagマッピング
        ├── glim_rosbag_docker_L2_outdoor_gpu.sh   // Unitree L2 屋外GPU rosbagマッピング
        ├── glim_rosbag_docker_L2_outdoor_cpu.sh   // Unitree L2 屋外CPU rosbagマッピング
        ├── glim_rosbag_docker_mid360_gpu.sh       // Livox Mid-360 GPU rosbagマッピング
        ├── glim_rosnode_docker_L2.sh              // リアルタイムROSノード（トピック購読）
        └── output/                                // SLAM出力データ
```

## 設定概要

### GPU vs CPU 設定

| タイプ | Dockerイメージ | 処理速度 |
|--------|----------------|----------|
| **GPU** | `koide3/glim_ros2:humble_cuda12.2` | 高速（1.5-3倍） |
| **CPU** | `koide3/glim_ros2:humble` | 中速（5-8倍） |

### 環境別最適化

| パラメータ | 屋内 | 屋外 | 説明 |
|------------|------|------|------|
| **ボクセル解像度** | 0.1-0.15m | 0.3-0.4m | 屋内は細かい詳細が必要 |
| **キーフレーム距離** | 0.3m | 1.5m | 屋内：頻繁更新<BR>屋外：疎更新 |
| **ループ検出** | 25-30m | 60-100m | 屋内：狭い空間<BR>屋外：広いエリア |
| **点群密度** | 35K-50K | 20K-30K | 屋内：詳細が必要 |
| **IMUノイズ** | 低 | 高 | 屋内：安定動作<BR>屋外：可変条件 |

## 使用方法

### [L2 サンプルデータ](https://www.unitree.com/download/L2)の保存場所

- [L2 屋内点群データ](https://oss-global-cdn.unitree.com/static/L2%20Indoor%20Point%20Cloud%20Data.bag): `$HOME/localization/unitree_L2/L2_Indoor_Point_Cloud_Data_sample_ROS2`
- [L2 公園点群データ](https://oss-global-cdn.unitree.com/static/L2%20Park%20Point%20Cloud%20Data.bag): `$HOME/localization/unitree_L2/L2_Park_Point_Cloud_Data_sample_ROS2`


### 屋内マッピング

シェルスクリプトで自動処理

```bash
# GPU
./glim_rosbag_docker_L2_indoor_gpu.sh $HOME/localization/unitree_L2/L2_Indoor_Point_Cloud_Data_sample_ROS2

# CPU
./glim_rosbag_docker_L2_indoor_cpu.sh $HOME/localization/unitree_L2/L2_Indoor_Point_Cloud_Data_sample_ROS2
```

### 屋外マッピング

シェルスクリプトで自動処理

```bash
# GPU
./glim_rosbag_docker_L2_outdoor_gpu.sh $HOME/localization/unitree_L2/L2_Park_Point_Cloud_Data_sample_ROS2

# CPU
./glim_rosbag_docker_L2_outdoor_cpu.sh $HOME/localization/unitree_L2/L2_Park_Point_Cloud_Data_sample_ROS2
```

### Livox Mid-360 マッピング

シェルスクリプトで自動処理

```bash
# GPU
./glim_rosbag_docker_mid360_gpu.sh /path/to/your/mid360_rosbag

# Livox Mid-360 センサー特性に最適化された設定
# トピック: /livox/lidar, /livox/imu
# T_lidar_imu: [-0.011, -0.02329, 0.04412, 0.0, 0.0, 0.0, 1.0]
```

### GLIMをオフラインビューアーモードで立ち上げ（構成データをGUIで確認・`.ply`で保存）

構成データをGUIで3D点群データとして確認し、`.PLY`形式で保存する。

```bash
# GPU
docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --gpus all \
  -e=DISPLAY \
  -e=NVIDIA_VISIBLE_DEVICES=all \
  -e=NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/glim_docker/config:/glim/config \
  -v $HOME/glim_docker/output:/tmp/dump \
  koide3/glim_ros2:humble_cuda12.2 \
  ros2 run glim_ros offline_viewer

# CPU
docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  -e=DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/glim_docker/config":/glim/config \
  -v "$HOME/glim_docker/output":/tmp/dump \
  koide3/glim_ros2:humble \
  ros2 run glim_ros offline_viewer
```

GLIMのGUIが立ち上がったら、左上のタブから`File->read`を選択し、`/tmp/dump/`の構成データを指定する。

> [!WARNING]
> GLIMはDocker環境で立ち上げているので、ローカル環境の`$HOME/glim_docker/output`ではなくDocker環境内の`/tmp/dump/`でないと構成データが見つからないことに注意

点群が3DのMAPになっていれば、`File -> Save -> Save Map`と選択して`.ply`形式で`/tmp/dump/`に保存すれば、ローカル環境の`$HOME/glim_docker/output`に点群MAPファイル保存される。

### GLIMをマップエディタモードで立ち上げ（ノイズの削除など）

[GLIM Manual Object Removal](https://koide3.github.io/glim/edit.html) を参考に、構成ファイルから手動でノイズ除去などの処理を実行できる。

```bash
# GPU
docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --gpus all \
  -e=DISPLAY \
  -e=NVIDIA_VISIBLE_DEVICES=all \
  -e=NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/glim_docker/config:/glim/config \
  -v $HOME/glim_docker/output:/tmp/dump \
  koide3/glim_ros2:humble_cuda12.2 \
  ros2 run glim_ros map_editor

# CPU
docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  -e=DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/glim_docker/config":/glim/config \
  -v "$HOME/glim_docker/output":/tmp/dump \
  koide3/glim_ros2:humble \
  ros2 run glim_ros map_editor
```

GLIMのGUIが立ち上がったら、左上のタブから`File->read`を選択し、`/tmp/dump/`の構成データを指定する。

> [!WARNING]
> GLIMはDocker環境で立ち上げているので、ローカル環境の`$HOME/glim_docker/output`ではなくDocker環境内の`/tmp/dump/`でないと構成データが見つからないことに注意


## 設定のコツ

### より高い精度が必要な場合

- **点群密度を増加**: `random_downsample_target` の値を高く
- **外れ値除去を有効化**: 前処理で `enable_outlier_removal: true` に設定
- **ボクセル解像度を減少**: より細かい詳細を捉えるため値を小さく
- **スムージングウィンドウを長く**: 軌道安定のため `smoother_lag` を増加

### より早い速度で処理する場合

- **GPU版を使用**: CUDA アクセラレーション付き
- **点群密度を減少**: `random_downsample_target` の値を低く
- **グローバル最適化を無効化**: グローバルマッピングで `enable_optimization: false` に設定
- **ボクセル解像度を増加**: 高速処理のため値を大きく
- **スムージングウィンドウを短く**: 迅速更新のため `smoother_lag` を減少

### よくある問題のトラブルシューティング

#### 低オーバーラップ警告

```
[global] [warning] previous submap has only a small overlap
```

**解決方法:**

- `min_implicit_loop_overlap` を減少（0.7 → 0.5）
- `max_implicit_loop_distance` を増加
- より頻繁なキーフレームのため `keyframe_delta_trans` と `keyframe_delta_rot` を調整

#### IndeterminantLinearSystemException

```
[global] [error] an indeterminant linear system exception was caught
```

**解決方法:**

- グローバルマッピングで `enable_optimization: false` に設定
- `init_pose_damping_scale` を増加（1e25 → 1e30）
- より保守的な `isam2_relinearize_thresh` 値を使用（0.001 → 0.01）

#### 軌道精度不良（点群ドリフト）

**解決方法:**

- より長い最適化ウィンドウのため `smoother_lag` を増加
- "GICP" の代わりに `registration_type: "VGICP"` を使用
- より頻繁な再線形化のため `isam2_relinearize_thresh` を減少
- より堅牢な最適化のため `use_isam2_dogleg: true` を有効化

### パフォーマンス監視

- **再生速度**: 処理速度のコンソール出力を監視（高い = 高速）
- **メモリ使用量**: 処理中のシステムメモリをチェック
- **サブマップオーバーラップ**: 連続するサブマップ間で70%超のオーバーラップを目指す
- **出力品質**: 生成された点群地図の完全性を検査
