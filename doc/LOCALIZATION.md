

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

Add ground truth map map in launch file
```yaml
parameters=[LaunchConfiguration("config_file"),
    { "calibration_file": LaunchConfiguration("calibration_file"),
     "map_dir": os.path.join(home_directory, "/path/to/your/pcd"),
}]
```
To quickly launch our localization module, feel free to try out this demo [dataset](https://drive.google.com/drive/folders/1WOTj4j9t5LkKkdajFlj6bZcdmPcsJipz?usp=sharing) using default initial pose configuration. 
