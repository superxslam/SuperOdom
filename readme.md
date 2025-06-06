# SuperOdometry: Lightweight LiDAR-inertial Odometry and Mapping

<div align="center">

[![Website](https://img.shields.io/badge/Website-4385f4?style=flat&logo=googlehome&logoColor=white)](https://superodometry.com/) [![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](./LICENSE)

</div>

<p align="center">
  <img src="./doc/superodom.png" alt="Super Odometry Pipeline" width="800"/>
</p>

> 🔥 This is a slim version of Super Odometry, containing the LiDAR Odometry component and IMU Odometry component. The LiDAR odometry only provides pose constraints to IMU odometry modules to estimate the bias of IMU. In return, the IMU Odometry module offers pose predictions to the LiDAR Odometry module, serving as an initial guess for ICP optimization.
<p align="center">
  <img src="./doc/tested_platform.png" alt="Super Odometry Pipeline" width="800"/>
</p>

> 🔥 The system has been widely tested on above platforms equipped with Livox, Velodyne and Ouster LiDAR. 

## 🔥 1. Key Features

- **Multi-LiDAR Support**
  - Compatible with Livox, Velodyne, and Ouster sensors
- **LiDAR-inertial Fusion**
  - Support LiDAR-inertial Fusion 
- **Dual-Mode Operation**
  - Supports both localization and mapping modes
- **Alignment Risk Prediction**
  - Provides alignment risk prediction for ICP algorithms
- **Degeneracy Awareness**
  - Robust detection of environmental degeneracy
- **ROS 2.0 Integration**
  - Built on ROS 2 Humble for modern robotics development

<p align="center">
  <img src="./doc/degradtion.png" alt="Super Odometry Pipeline" width="800"/>
</p>

<p align="center">
  <img src="./doc/uncertainty.gif" alt="Alignment Risk Prediction" width="800"/>
</p>

> 🔥 6 DOF degeneracy uncertainty detection. We support visualization in both RVIZ and Rerun. 

## 📦 3. Installation
> Highly recommend to check our docker files to run our code with step 4 and step 5. 
### System Requirements

- ROS2 Humble
- PCL
- Eigen
- [Sophus](https://github.com/strasdat/Sophus)
- [GTSAM (4.0.2 or 4.1)](https://github.com/borglab/gtsam)
- [Ceres Solver](http://ceres-solver.org/)

### Dependencies Installation

#### Install Sophus
```bash
git clone http://github.com/strasdat/Sophus.git
cd Sophus && git checkout 97e7161
mkdir build && cd build
cmake .. -DBUILD_TESTS=OFF
make -j8 && sudo make install
```

#### Install GTSAM
```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam && git checkout 4abef92
mkdir build && cd build
cmake \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  ..
make -j6 && sudo make install
```

#### Install Ceres
```bash
sudo apt-get install -y libceres-dev
```

#### Install Rerun
```bash
pip install rerun-sdk
```

## 🐳 4. Docker Setup

### Prerequisites
- [Docker](https://www.docker.com/)
- [NVIDIA Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

### Building Docker Image
```bash
cd ros2_humble_docker
docker build -t superodom-ros2:latest .
```

### Workspace Structure

First create your own local ROS2 workspace and clone `SuperOdom`: 
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/superxslam/SuperOdom
```
Clone respective repos and ensure they follow this exact structure under `ros2_ws/src`:
```
ros2_ws/src
├── SuperOdom
├── livox_ros_driver2
└── rviz_2d_overlay_plugins
```
You can clone `livox_ros_driver2` and `rviz_2d_overlay_plugins` using the following link:

- [Livox-ROS-driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- [ROS2-jsk-plugin](https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins)

> **Important**: Maintain this exact structure within `ros_ws/src`

### Docker Container Setup
```bash
# Allow Docker GUI access
xhost +local:docker
```

Go to `ros2_humble_docker/container_run.sh` and make sure you change exact directory path for `PROJECT_DIR` and `DATASET_DIR`
```bash
PROJECT_DIR="/path/to/your/superodom"
DATASET_DIR="/path/to/your/dataset"
```
> **Important**: `PROJECT_DIR` should be the exact directory to `ros2_ws/src`

Then launch docker container using the following:
```bash
# Grant access
cd ros2_humble_docker
sudo chmod -R 777 container_run.sh

# Start container
./container_run.sh superodom-ros2 superodom-ros2:latest

# Source ROS2
source /opt/ros/humble/setup.bash
```
> **Important**: To access container, you can open a new bash window and run `docker exec --privileged -it superodom-ros2 /bin/bash` 

Build the workspace within container
```bash
cd ~/ros2_ws/src/livox_ros_driver2
./build.sh humble 
cd ~/ros2_ws
colcon build
```
> **Important**: make sure you first build `livox_ros_driver2` 

## 🚀 5. Launch SuperOdometry

To launch SuperOdometry, we provide demo datasets for Livox-mid360, VLP-16 and OS1-128 sensor [Download Link](https://drive.google.com/drive/folders/1oA0kRFIH0_8oyD32IW1vZitfxYunzdBr?usp=sharing)  

For more challange dataset, feel free to download from our website [slam_mode](https://superodometry.com/iccv23_challenge_LiI) and [localization_mode](https://superodometry.com/superloc). You might want to convert ROS1 bag into ROS2 format using this [link](https://docs.openvins.com/dev-ros1-to-ros2.html). 

For user-defined topic names, modify `super_odometry/config/$(YOUR_LiDAR_SENSOR).yaml`: 
```bash
imu_topic: "/your/imu/topic"
laser_topic: "/your/laser/topic"
```
For user-defined laser-imu extrinsics, modify `super_odometry/config/$(YOUR_LiDAR_SENSOR)/$(YOUR_LiDAR_SENSOR)_calibration.yaml`: 
```bash
#Rotation from laser frame to imu frame, imu^R_laser
extrinsicRotation_imu_laser: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d  
  data: [1., 0., 0.,
        0., 1., 0.,
        0., 0., 1.]

#Translation from laser frame to imu frame, imu^T_laser
extrinsicTranslation_imu_laser: !!opencv-matrix
  rows: 3
  cols: 1
  dt: d
  data: [-0.011, -0.02329, 0.04412]
```

Run SuperOdometry using the following command: 

```bash
source install/setup.bash
ros2 launch super_odometry livox_mid360.launch.py
ros2 launch super_odometry os1_128.launch.py
ros2 launch super_odometry vlp_16.launch.py
```
Play your ROS2 dataset:
```bash
# launch this in a new bash window
docker exec --privileged -it superodom-ros2 /bin/bash
source install/setup.bash
cd ~/data
ros2 play $(YOUR_ROS2_DATASET)
```

Visualize in RVIZ2: 
```bash
# launch this in a new bash window
docker exec --privileged -it superodom-ros2 /bin/bash
source install/setup.bash
cd ~/ros_ws/src/SuperOdom/super_odometry
rviz2 -d ros2.rviz
```

(⭐ Alternative) Visualize in Rerun: 
```bash
# launch this in a new bash window
docker exec --privileged -it superodom-ros2 /bin/bash
source install/setup.bash
cd ~/ros2_ws/src/SuperOdom/script/visualizers
python3 rerun_visualizer.py
# Open a new bash window on your local device
rerun
```

We also provide tmux script for easy launch with dataset (this script only works after you build the workspace in docker): 
```bash
cd script
tmuxp load run.yaml
```

## 📍 Localization Mode Configuration

https://github.com/user-attachments/assets/42cb5480-c283-4608-84be-ff12a05d09e0

> 🔥 The localization mode allows you to localize your robot by providing an initial pose and ground truth map. 

Update your `super_odometry/config/$(YOUR_LiDAR_SENSOR).yaml` configuration file with:
```yaml
localization_mode: true         # If true, localization mode is enabled; otherwise, SLAM mode is used
read_pose_file: false           # Set to true to read initial pose from a txt file
init_x: 0.0                     # Initial X position for localization
init_y: 0.0                     # Initial Y position for localization
init_z: 0.0                     # Initial Z position for localization
init_roll: 0.0                  # Initial roll angle
init_pitch: 0.0                 # Initial pitch angle
init_yaw: 0.0                   # Initial yaw angle
```

Add ground truth map map in launch file
```yaml
parameters=[LaunchConfiguration("config_file"),
    { "calibration_file": LaunchConfiguration("calibration_file"),
     "map_dir": os.path.join(home_directory, "/path/to/your/pcd"),
}]
```
To quickly launch our localization module, feel free to try out this demo [dataset](https://drive.google.com/drive/folders/1WOTj4j9t5LkKkdajFlj6bZcdmPcsJipz?usp=sharing) using default initial pose configuration. 

<!-- ## 📫 7. Contact

- [Open an Issue](https://github.com/YourUsername/SuperOdometry)
- [Visit our Website](https://superodometry.com/contact) -->

## 📚 8. Citations

```bibtex
@inproceedings{zhao2021super,
  title={Super odometry: IMU-centric LiDAR-visual-inertial estimator for challenging environments},
  author={Zhao, Shibo and Zhang, Hengrui and Wang, Peng and Nogueira, Lucas and Scherer, Sebastian},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={8729--8736},
  year={2021},
  organization={IEEE}
}

@inproceedings{zhao2025superloc,
  title={SuperLoc: The Key to Robust LiDAR-Inertial Localization Lies in Predicting Alignment Risks},
  author={Zhao, Shibo and Zhu, Honghao and Gao, Yuanjun and Kim, Beomsoo and Qiu, Yuheng and Johnson, Aaron M. and Scherer, Sebastian},
  booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2025},
  url={https://arxiv.org/abs/2412.02901}
}
```

## 9. Next Plan
🔵 Colorized Point Cloud Visualization — [Video Demo](https://www.youtube.com/watch?v=r7nLDGrz4gE)

🟢 Visual Odometry Module — Initial Release
Lightweight and robust visual odometry module integrated into SuperOdometry.


## 📝 10. License

This package is released under the GPLv3 license. For commercial use, please contact shiboz@andrew.cmu.edu and Prof. Sebastian Scherer.

## 🙏 11. Acknowledgements

Special thanks to Professor Ji Zhang, Professor Michael Kaess, Parv Maheshwari, Yuanjun Gao, Yaoyu Hu for their valuable advice. Thanks to Omar Alama for providing Rerun support. We also acknowledge these foundational works:

- LOAM: Lidar Odometry and Mapping in Real-time (RSS 2014)
- GTSAM: Georgia Tech Smoothing and Mapping Library
- [FastLIO](https://github.com/hku-mars/FAST_LIO),  [LIOSAM](https://github.com/TixiaoShan/LIO-SAM)
