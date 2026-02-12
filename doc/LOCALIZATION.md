

## 📍 Localization Mode Configuration


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

Add ground truth map map in launch file is 
```yaml
parameters=[LaunchConfiguration("config_file"),
    { "calibration_file": LaunchConfiguration("calibration_file"),
     "map_dir": os.path.join(home_directory, "/path/to/your/pcd"),
}]
```
The default limit for groudtruth map size is 21 x 21 x 11 array of 50m voxels, so the total spatial coverage is:
```yaml
X: init_x ± 525m (10.5 voxels × 50m each side)
Y: init_y ± 525m
Z: init_z ± 275m             
```
If your ground truth map is larger than above, you can modified the parameter under `LocalMap.h`
```bash
static constexpr const int laserCloudWidth = 21;
static constexpr const int laserCloudHeight = 21;
static constexpr int laserCloudDepth = 11;
static constexpr int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;  // 4851
static constexpr double voxelResulation = 50;
```

To quickly launch our localization module, feel free to try out this demo [dataset](https://drive.google.com/drive/folders/1WOTj4j9t5LkKkdajFlj6bZcdmPcsJipz?usp=sharing) using default initial pose configuration. 
