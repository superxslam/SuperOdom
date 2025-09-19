#pragma once

#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "super_odometry/utils/Twist.h"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

enum class SensorType {VELODYNE, OUSTER, LIVOX};
extern std::string IMU_TOPIC;
extern std::string LASER_TOPIC;
extern std::string ODOM_TOPIC;
extern std::string DepthUP_TOPIC;
extern std::string DepthDown_TOPIC;
extern std::string ProjectName;

extern std::string WORLD_FRAME;
extern std::string WORLD_FRAME_ROT;
extern std::string SENSOR_FRAME;
extern std::string SENSOR_FRAME_ROT;
extern SensorType sensor;

extern int PROVIDE_IMU_LASER_EXTRINSIC;

extern std::vector<Eigen::Matrix3d> RIC;

extern std::vector<Eigen::Vector3d> TIC;

extern Eigen::Matrix3d imu_laser_R;

extern Eigen::Vector3d imu_laser_T;

extern Eigen::Matrix3d cam_laser_R;

extern Eigen::Vector3d cam_laser_T;

extern Eigen::Matrix3d imu_camera_R;

extern Eigen::Vector3d imu_camera_T;

extern Eigen::Vector3d imu_laser_offset;

extern Transformd Tcam_lidar;

extern Transformd T_i_c;

extern Transformd T_i_l;

extern Transformd T_l_i;

extern float up_realsense_roll;

extern float up_realsense_pitch;

extern float up_realsense_yaw;

extern float up_realsense_x;

extern float up_realsense_y;

extern float up_realsense_z;

extern float down_realsense_roll;

extern float down_realsense_pitch;

extern float down_realsense_yaw;

extern float down_realsense_x;

extern float down_realsense_y;

extern float down_realsense_z;

extern float yaw_ratio;

extern float IMU_ACC_X_LIMIT;

extern float IMU_ACC_Y_LIMIT;

extern float IMU_ACC_Z_LIMIT;

extern bool USE_IMU_ROLL_PITCH;

extern bool SAVE_PLY;

extern std::string SENSOR; 

extern Transformd T_ouster_sensor;

extern Eigen::Matrix3d ouster_sensor_R;

extern Eigen::Vector3d ouster_sensor_T;

bool readGlobalparam(rclcpp::Node::SharedPtr);

bool readCalibration(rclcpp::Node::SharedPtr);

