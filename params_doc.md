# Documenting Params and how they affect the performance.

<details>
<summary> ** Laser Mapping Params** </summary>

1. mapping_line_resolution → config_.lineRes

```
downSizeFilterCorner.setLeafSize(config_.lineRes, config_.lineRes, config_.lineRes);
slam.localMap.lineRes_ = config_.lineRes;

```
Effect: Voxel size for edge/corner features. Bigger → fewer edge points, faster but less detail.



2. mapping_plane_resolution → config_.planeRes

```
downSizeFilterSurf.setLeafSize(config_.planeRes, config_.planeRes, config_.planeRes);
slam.localMap.planeRes_ = config_.planeRes;
```

Effect: Voxel size for surface features. Bigger → faster but coarser planes.


3. max_iterations → config_.max_iterations

```
slam.LocalizationICPMaxIter = config_.max_iterations;
```

Effect: Max ICP/opt iters. Higher → better fit (sometimes), slower.


4. debug_view → config_.debug_view_enabled

```
if (frameCount % 5 == 0 && config_.debug_view_enabled) {
  *laserCloudSurround = slam.localMap.get5x5LocalMap(...);
  pubLaserCloudSurround->publish(...);
}
```

Effect: Publishes local map periodically for visualization; small CPU overhead.


5. use_imu_roll_pitch → config_.use_imu_roll_pitch

```
// note: value taken from macro USE_IMU_ROLL_PITCH
if (config_.use_imu_roll_pitch) {
  slam.OptSet.use_imu_roll_pitch = true;
  slam.OptSet.imu_roll_pitch = utils::extractRollPitch(sensorMeas.imuPrediction);
} else {
  slam.OptSet.use_imu_roll_pitch = false;
}
```

Effect: Constrains/regularizes roll & pitch from IMU; stabilizes in sparse scenes.  

  



. auto_voxel_size

```
if (config_.auto_voxel_size) { /* compute average-dist */ 
  if (slam.stats.average_distance < 25){ config_.lineRes=0.1; config_.planeRes=0.2; }
  else if (slam.stats.average_distance > 65){ config_.lineRes=0.4; config_.planeRes=0.8; }
  downSizeFilterSurf.setLeafSize(config_.planeRes,...);
  downSizeFilterCorner.setLeafSize(config_.lineRes,...);
}
```
4.


</details>