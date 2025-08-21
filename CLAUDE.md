# GLIM Docker

This project runs [GLIM](https://koide3.github.io/glim/) ([github: koide3/glim_ros2](https://github.com/koide3/glim_ros2)), which creates a point cloud map using LiDAR SLAM, in a Docker container environment.

## Environment

### Supported LiDAR Sensors

#### Unitree D4 LiDAR L2
- [Unitree D4 LiDAR L2](https://www.unitree.com/L2)
  - SKD: https://github.com/unitreerobotics/unilidar_sdk2
  - ROS2 TOPIC
    - LiDAR: /unilidar/cloud
    - IMU:  /unilidar/imu
  - "T_lidar_imu": [0.007698, 0.014655, -0.00667, 0.0, 0.0, 0.0, 1.0],
  - Scanning distance: 30m / 90%
  - Nearbyblind area: 0.05m
  - Effective frequency: 64000points/s
  - Circumferential scanning frequency: 5.55Hz
  - IMU: 3-axis acceleration +3-axis gyroscope

#### Livox Mid-360
- [Livox Mid-360](https://www.livoxtech.com/mid-360)
  - ROS2 TOPIC
    - LiDAR: /livox/lidar
    - IMU: /livox/imu
  - "T_lidar_imu": [-0.011, -0.02329, 0.04412, 0.0, 0.0, 0.0, 1.0]
  - Detection range: 70m @ 80% reflectivity
  - FOV: 360° × 59°
  - Scan rate: 10Hz
  - IMU: 6-axis (3-axis accelerometer + 3-axis gyroscope)

### PC Environment
- GPU: NVIDIA Geforce RTX 5060Ti

## Environment construction (Reference source: https://koide3.github.io/glim/docker.html)

### With GPU

```bash
# Copy config and edit as you want
git clone https://github.com/koide3/glim.git /tmp/glim
cp -R /tmp/glim/config $HOME/glim_docker/config

# Pull image from docker hub
docker pull koide3/glim_ros2:humble_cuda12.2

# Launch glim_ros2:humble_cuda12.2 image with GPU and DISPLAY support
docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --pid=host \
  --gpus all \
  -e=DISPLAY \
  -e=ROS_DOMAIN_ID \
  -v $(realpath config):/glim/config \
  koide3/glim_ros2:humble_cuda12.2 \
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config
```

## Without GPU

```bash
# Copy config and edit it
git clone https://github.com/koide3/glim.git /tmp/glim
cp -R /tmp/glim/config $HOME/glim_docker/config

# Change as follows:
# "config_odometry" : "config_odometry_cpu.json"
# "config_sub_mapping" : "config_sub_mapping_cpu.json"
# "config_global_mapping" : "config_global_mapping_cpu.json"
nano config/config.json

# Pull image from docker hub
docker pull koide3/glim_ros2:humble

# Launch glim_ros2:humble image with DISPLAY support
docker run \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --pid=host \
  --gpus all \
  -e=DISPLAY \
  -e=ROS_DOMAIN_ID \
  -v $(realpath config):/glim/config \
  koide3/glim_ros2:humble \
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config
```

### Build docker images from source

```bash
mkdir /tmp/glim_docker && cd /tmp/glim_docker
git clone https://github.com/koide3/glim.git 
git clone https://github.com/koide3/glim_ros2.git

# Without GPU
docker build \
  -f glim_ros2/docker/Dockerfile.gcc \
  --build-arg="BASE_IMAGE=koide3/gtsam_points:jammy" \
  --build-arg="ROS_DISTRO=humble" \
  --tag glim_ros2:humble \
  .

# With GPU
docker build \
  -f glim_ros2/docker/Dockerfile.gcc.cuda \
  --build-arg="BASE_IMAGE=koide3/gtsam_points:jammy_cuda12.2" \
  --build-arg="ROS_DISTRO=humble" \
  --tag glim_ros2:humble_cuda12.2 \
```

## [L2 Sample Data](https://www.unitree.com/download/L2)

- [L2 Indoor Observed Point Cloud Data](https://oss-global-cdn.unitree.com/static/L2%20Indoor%20Point%20Cloud%20Data.bag): `$HOME/localization/unitree_L2/L2_Indoor_Point_Cloud_Data_sample_ROS2`
- [L2 Park Observed Point Cloud Data](https://oss-global-cdn.unitree.com/static/L2%20Park%20Point%20Cloud%20Data.bag): `$HOME/localization/unitree_L2/L2_Park_Point_Cloud_Data_sample_ROS2`

## Directory structure

```
$HOME
└── glim_docker/
        ├── config/                                // Default Settings (Original GLIM)
        ├── config_L2_default/                     // Unitree L2 Default Settings
        ├── config_L2_indoor_gpu/                  // Unitree L2 Indoor GPU Settings (High-speed & High-precision)
        ├── config_L2_indoor_cpu/                  // Unitree L2 Indoor CPU Settings (High-precision)
        ├── config_L2_outdoor_gpu/                 // Unitree L2 Outdoor GPU Settings (High-speed & High-precision)
        ├── config_L2_outdoor_cpu/                 // Unitree L2 Outdoor CPU Settings (High-precision)
        ├── config_mid360/                         // Livox Mid-360 Settings
        ├── glim_offline_viewer_docker_cpu.sh      // Offline Viewer (CPU)
        ├── glim_offline_viewer_docker_gpu.sh      // Offline Viewer (GPU)
        ├── glim_rosbag_docker_L2_indoor_gpu.sh    // Unitree L2 Indoor GPU mapping from rosbag
        ├── glim_rosbag_docker_L2_indoor_cpu.sh    // Unitree L2 Indoor CPU mapping from rosbag
        ├── glim_rosbag_docker_L2_outdoor_gpu.sh   // Unitree L2 Outdoor GPU mapping from rosbag
        ├── glim_rosbag_docker_L2_outdoor_cpu.sh   // Unitree L2 Outdoor CPU mapping from rosbag
        ├── glim_rosbag_docker_mid360_gpu.sh       // Livox Mid-360 GPU mapping from rosbag
        ├── glim_rosnode_docker_L2.sh              // Real-time ROS node (subscribes to topics)
        └── output/                                // SLAM Output Data
```

## Configuration Overview

### GPU vs CPU Configurations

| Type | Environment | Docker Image | Processing Speed | Accuracy | Use Case |
|------|-------------|--------------|------------------|----------|----------|
| **GPU** | Indoor/Outdoor | `koide3/glim_ros2:humble_cuda12.2` | Fast (1.5-3x) | High | Real-time processing, Quick mapping |
| **CPU** | Indoor/Outdoor | `koide3/glim_ros2:humble` | Moderate (5-8x) | Very High | Offline processing, Maximum precision |

### Environment-Specific Optimizations

| Parameter | Indoor | Outdoor | Description |
|-----------|--------|---------|-------------|
| **Voxel Resolution** | 0.1-0.15m | 0.3-0.4m | Indoor needs finer details |
| **Keyframe Distance** | 0.3m | 1.5m | Indoor: frequent updates, Outdoor: sparse updates |
| **Loop Detection** | 25-30m | 60-100m | Indoor: smaller spaces, Outdoor: larger areas |
| **Point Cloud Density** | 35K-50K | 20K-30K | Indoor: more detail needed |
| **IMU Noise** | Lower | Higher | Indoor: stable motion, Outdoor: variable conditions |

## Usage Instructions

### Indoor Mapping

```bash
# GPU Version (Recommended for real-time)
./glim_rosbag_docker_L2_indoor_gpu.sh $HOME/localization/unitree_L2/L2_Indoor_Point_Cloud_Data_sample_ROS2

# CPU Version (Maximum precision)
./glim_rosbag_docker_L2_indoor_cpu.sh $HOME/localization/unitree_L2/L2_Indoor_Point_Cloud_Data_sample_ROS2
```

### Outdoor Mapping

```bash
# GPU Version (Recommended for real-time)
./glim_rosbag_docker_L2_outdoor_gpu.sh $HOME/localization/unitree_L2/L2_Park_Point_Cloud_Data_sample_ROS2

# CPU Version (Maximum precision)
./glim_rosbag_docker_L2_outdoor_cpu.sh $HOME/localization/unitree_L2/L2_Park_Point_Cloud_Data_sample_ROS2
```

### Livox Mid-360 Mapping

```bash
# GPU Version (Recommended for real-time)
./glim_rosbag_docker_mid360_gpu.sh /path/to/your/mid360_rosbag

# Configuration optimized for Livox Mid-360 sensor characteristics
# Topics: /livox/lidar, /livox/imu
# T_lidar_imu: [-0.011, -0.02329, 0.04412, 0.0, 0.0, 0.0, 1.0]
```

## Configuration Tips

### For Better Accuracy

- **Use CPU versions** for offline processing when maximum precision is needed
- **Increase point cloud density**: Higher `random_downsample_target` values
- **Enable outlier removal**: Set `enable_outlier_removal: true` in preprocessing
- **Reduce voxel resolution**: Smaller values for finer detail capture
- **Longer smoothing window**: Increase `smoother_lag` for more stable trajectories

### For Better Speed

- **Use GPU versions** with CUDA acceleration
- **Reduce point cloud density**: Lower `random_downsample_target` values
- **Disable global optimization**: Set `enable_optimization: false` in global mapping
- **Increase voxel resolution**: Larger values for faster processing
- **Shorter smoothing window**: Decrease `smoother_lag` for quicker updates

### Troubleshooting Common Issues

#### Low Overlap Warnings

```
[global] [warning] previous submap has only a small overlap
```

**Solutions:**

- Decrease `min_implicit_loop_overlap` (0.7 → 0.5)
- Increase `max_implicit_loop_distance`
- Adjust `keyframe_delta_trans` and `keyframe_delta_rot` for more frequent keyframes

#### IndeterminantLinearSystemException

```
[global] [error] an indeterminant linear system exception was caught
```

**Solutions:**

- Set `enable_optimization: false` in global mapping
- Increase `init_pose_damping_scale` (1e25 → 1e30)
- Use more conservative `isam2_relinearize_thresh` values (0.001 → 0.01)

#### Poor Trajectory Accuracy (Point Cloud Drift)

**Solutions:**

- Increase `smoother_lag` for longer optimization windows
- Use `registration_type: "VGICP"` instead of "GICP"
- Reduce `isam2_relinearize_thresh` for more frequent relinearization
- Enable `use_isam2_dogleg: true` for more robust optimization

### Performance Monitoring

- **Playback Speed**: Monitor console output for processing speed (higher = faster)
- **Memory Usage**: Check system memory during processing
- **Submap Overlap**: Aim for >70% overlap between consecutive submaps
- **Output Quality**: Inspect generated point cloud maps for completeness
