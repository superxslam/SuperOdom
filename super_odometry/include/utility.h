#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>

//#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

using namespace std;

typedef pcl::PointXYZI PointType;

class ParamServer {
public:
  rclcpp::Node::SharedPtr node;

  std::string robot_id;

  string pointCloudTopic;
  string imuTopic;
  string odomTopic;
  string gpsTopic;

  // GPS Settings
  bool useImuHeadingInitialization;
  bool useGpsElevation;
  float gpsCovThreshold;
  float poseCovThreshold;

  // Save pcd
  bool savePCD;
  string savePCDDirectory;

  // Velodyne Sensor Configuration: Velodyne
  int N_SCAN;
  int Horizon_SCAN;

  // IMU
  float imuAccNoise;
  float imuGyrNoise;
  float imuAccBiasN;
  float imuGyrBiasN;
  float imuGravity;

  vector<double> extRimu_camV;
  vector<double> extTranimu_camV;
  vector<double> extRcam_laserV;
  vector<double> extTrancam_laserV;

  vector<double> extRotV;
  vector<double> extRPYV;
  vector<double> extTransV;

  Eigen::Matrix3d extRimu_cam;
  Eigen::Vector3d extTranimu_cam;
  Eigen::Matrix3d extRcam_laser;
  Eigen::Vector3d extTrancam_laser;

  Eigen::Matrix3d extRot;
  Eigen::Matrix3d extRPY;
  Eigen::Vector3d extTrans;
  Eigen::Quaterniond extQRPY;

  // LOAM
  float edgeThreshold;
  float surfThreshold;
  int edgeFeatureMinValidNum;
  int surfFeatureMinValidNum;

  // voxel filter paprams
  float odometrySurfLeafSize;
  float mappingCornerLeafSize;
  float mappingSurfLeafSize;

  float z_tollerance;
  float rotation_tollerance;

  // CPU Params
  int numberOfCores;
  double mappingProcessInterval;

  // Surrounding map
  float surroundingkeyframeAddingDistThreshold;
  float surroundingkeyframeAddingAngleThreshold;
  float surroundingKeyframeDensity;
  float surroundingKeyframeSearchRadius;

  // Loop closure
  bool loopClosureEnableFlag;
  int surroundingKeyframeSize;
  float historyKeyframeSearchRadius;
  float historyKeyframeSearchTimeDiff;
  int historyKeyframeSearchNum;
  float historyKeyframeFitnessScore;

  // global map visualization radius
  float globalMapVisualizationSearchRadius;
  float globalMapVisualizationPoseDensity;
  float globalMapVisualizationLeafSize;

  ParamServer() {
    node->declare_parameter("smalldrone_id", "small_drone");

    node->declare_parameter("super_loam/pointCloudTopic", "points_raw");
    node->declare_parameter("super_loam/imuTopic", "/imu/data");
    node->declare_parameter("super_loam/odomTopic", "odometry/imu");
    node->declare_parameter("super_loam/gpsTopic", "odometry/gps");

    node->declare_parameter("super_loam/useImuHeadingInitialization",false);
    node->declare_parameter("super_loam/useGpsElevation", false);
    node->declare_parameter("super_loam/gpsCovThreshold", 2.0);
    node->declare_parameter("super_loam/poseCovThreshold", 25.0);

    node->declare_parameter("super_loam/savePCD", false);
    node->declare_parameter("super_loam/savePCDDirectory", "/Downloads/LOAM/");

    node->declare_parameter("super_loam/N_SCAN",  16);
    node->declare_parameter("super_loam/Horizon_SCAN", 1800);

    node->declare_parameter("super_loam/imuAccNoise",  0.01);
    node->declare_parameter("super_loam/imuGyrNoise",  0.001);
    node->declare_parameter("super_loam/imuAccBiasN",  0.0002);
    node->declare_parameter("super_loam/imuGyrBiasN",  0.00003);
    node->declare_parameter("super_loam/imuGravity",  9.80511);
    node->declare_parameter("super_loam/extrinsicRot",  vector<double>());
    node->declare_parameter("super_loam/extrinsicRPY", vector<double>());
    node->declare_parameter("super_loam/extrinsicTrans", vector<double>());

    node->declare_parameter("super_loam/extrinsicRotation", vector<double>());
    node->declare_parameter("super_loam/extrinsicTranslation", vector<double>());
    node->declare_parameter("super_loam/extrinsicRotation_camera_laser", vector<double>());
    node->declare_parameter("super_loam/extrinsicTranslation_camera_laser",vector<double>());

    node->declare_parameter("super_loam/edgeThreshold",  0.1);
    node->declare_parameter("super_loam/surfThreshold",  0.1);
    node->declare_parameter("super_loam/edgeFeatureMinValidNum", 10);
    node->declare_parameter("super_loam/surfFeatureMinValidNum", 100);

    node->declare_parameter("super_loam/odometrySurfLeafSize", 0.2);
    node->declare_parameter("super_loam/mappingCornerLeafSize", 0.2);
    node->declare_parameter("super_loam/mappingSurfLeafSize",  0.2);

    node->declare_parameter("super_loam/z_tollerance", FLT_MAX);
    node->declare_parameter("super_loam/rotation_tollerance", FLT_MAX);

    node->declare_parameter("super_loam/numberOfCores", 2);
    node->declare_parameter("super_loam/mappingProcessInterval", 0.15);

    node->declare_parameter("super_loam/surroundingkeyframeAddingDistThreshold", 1.0);
    node->declare_parameter("super_loam/surroundingkeyframeAddingAngleThreshold",0.2);
    node->declare_parameter("super_loam/surroundingKeyframeDensity",1.0);
    node->declare_parameter("super_loam/surroundingKeyframeSearchRadius", 50.0);

    node->declare_parameter("super_loam/loopClosureEnableFlag",  false);
    node->declare_parameter("super_loam/surroundingKeyframeSize", 50);
    node->declare_parameter("super_loam/historyKeyframeSearchRadius",10.0);
    node->declare_parameter("super_loam/historyKeyframeSearchTimeDiff",30.0);
    node->declare_parameter("super_loam/historyKeyframeSearchNum", 25);
    node->declare_parameter("super_loam/historyKeyframeFitnessScore", 0.3);

    node->declare_parameter("super_loam/globalMapVisualizationSearchRadius", 1e3);
    node->declare_parameter("super_loam/globalMapVisualizationPoseDensity", 10.0);
    node->declare_parameter("super_loam/globalMapVisualizationLeafSize", 1.0);

    robot_id = node->get_parameter("smalldrone_id").as_string();

    pointCloudTopic = node->get_parameter("super_loam/pointCloudTopic").as_string();
    imuTopic = node->get_parameter("super_loam/imuTopic").as_string();
    odomTopic = node->get_parameter("super_loam/odomTopic").as_string();
    gpsTopic = node->get_parameter("super_loam/gpsTopic").as_string();

    useImuHeadingInitialization = node->get_parameter("super_loam/useImuHeadingInitialization").as_bool();
    useGpsElevation = node->get_parameter("super_loam/useGpsElevation").as_bool();
    gpsCovThreshold = node->get_parameter("super_loam/gpsCovThreshold").as_double();
    poseCovThreshold = node->get_parameter("super_loam/poseCovThreshold").as_double();

    savePCD = node->get_parameter("super_loam/savePCD").as_bool();
    savePCDDirectory = node->get_parameter("super_loam/savePCDDirectory").as_string();

    N_SCAN = node->get_parameter("super_loam/N_SCAN").as_int();
    Horizon_SCAN = node->get_parameter("super_loam/Horizon_SCAN").as_int();

    imuAccNoise = node->get_parameter("super_loam/imuAccNoise").as_double();
    imuGyrNoise = node->get_parameter("super_loam/imuGyrNoise").as_double();
    imuAccBiasN = node->get_parameter("super_loam/imuAccBiasN").as_double();
    imuGyrBiasN = node->get_parameter("super_loam/imuGyrBiasN").as_double();
    imuGravity = node->get_parameter("super_loam/imuGravity").as_double();
    extRotV = node->get_parameter("super_loam/extrinsicRot").as_double_array();
    extRPYV = node->get_parameter("super_loam/extrinsicRPY").as_double_array();
    extTransV = node->get_parameter("super_loam/extrinsicTrans").as_double_array();

    extRimu_camV = node->get_parameter("super_loam/extrinsicRotation").as_double_array();
    extTranimu_camV = node->get_parameter("super_loam/extrinsicTranslation").as_double_array();
    extRcam_laserV = node->get_parameter("super_loam/extrinsicRotation_camera_laser").as_double_array();
    extTrancam_laserV = node->get_parameter("super_loam/extrinsicTranslation_camera_laser").as_double_array();

    edgeThreshold = node->get_parameter("super_loam/edgeThreshold").as_double();
    surfThreshold = node->get_parameter("super_loam/surfThreshold").as_double();
    edgeFeatureMinValidNum = node->get_parameter("super_loam/edgeFeatureMinValidNum").as_int();
    surfFeatureMinValidNum = node->get_parameter("super_loam/surfFeatureMinValidNum").as_int();

    odometrySurfLeafSize = node->get_parameter("super_loam/odometrySurfLeafSize").as_double();
    mappingCornerLeafSize = node->get_parameter("super_loam/mappingCornerLeafSize").as_double();
    mappingSurfLeafSize = node->get_parameter("super_loam/mappingSurfLeafSize").as_double();

    z_tollerance = node->get_parameter("super_loam/z_tollerance").as_double();
    rotation_tollerance = node->get_parameter("super_loam/rotation_tollerance").as_double();

    numberOfCores = node->get_parameter("super_loam/numberOfCores").as_int();
    mappingProcessInterval = node->get_parameter("super_loam/mappingProcessInterval").as_double();

    surroundingkeyframeAddingDistThreshold = node->get_parameter("super_loam/surroundingkeyframeAddingDistThreshold").as_double();
    surroundingkeyframeAddingAngleThreshold = node->get_parameter("super_loam/surroundingkeyframeAddingAngleThreshold").as_double();
    surroundingKeyframeDensity = node->get_parameter("super_loam/surroundingKeyframeDensity").as_double();
    surroundingKeyframeSearchRadius = node->get_parameter("super_loam/surroundingKeyframeSearchRadius").as_double();

    loopClosureEnableFlag = node->get_parameter("super_loam/loopClosureEnableFlag").as_bool();
    surroundingKeyframeSize = node->get_parameter("super_loam/surroundingKeyframeSize").as_int();
    historyKeyframeSearchRadius = node->get_parameter("super_loam/historyKeyframeSearchRadius").as_double();
    historyKeyframeSearchTimeDiff = node->get_parameter("super_loam/historyKeyframeSearchTimeDiff").as_double();
    historyKeyframeSearchNum = node->get_parameter("super_loam/historyKeyframeSearchNum").as_int();
    historyKeyframeFitnessScore = node->get_parameter("super_loam/historyKeyframeFitnessScore").as_double();

    globalMapVisualizationSearchRadius = node->get_parameter("super_loam/globalMapVisualizationSearchRadius").as_double();
    globalMapVisualizationPoseDensity = node->get_parameter("super_loam/globalMapVisualizationPoseDensity").as_double();
    globalMapVisualizationLeafSize = node->get_parameter("super_loam/globalMapVisualizationLeafSize").as_double();

    extRimu_cam =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            extRimu_camV.data(), 3, 3);

    extTranimu_cam =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            extTranimu_camV.data(), 3, 1);

    extRcam_laser =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            extRcam_laserV.data(), 3, 3);

    extTrancam_laser =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            extTrancam_laserV.data(), 3, 1);

    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        extRotV.data(), 3, 3);
    extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        extRPYV.data(), 3, 3);
    extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        extTransV.data(), 3, 1);
    extQRPY = Eigen::Quaterniond(extRPY);

  

    // nh.param<std::string>("/smalldrone_id", robot_id, "small_drone");

    // nh.param<std::string>("super_loam/pointCloudTopic", pointCloudTopic,
    //                       "points_raw");
    // nh.param<std::string>("super_loam/imuTopic", imuTopic, "/imu/data");
    // nh.param<std::string>("super_loam/odomTopic", odomTopic, "odometry/imu");
    // nh.param<std::string>("super_loam/gpsTopic", gpsTopic, "odometry/gps");

    // nh.param<bool>("super_loam/useImuHeadingInitialization",
    //                useImuHeadingInitialization, false);
    // nh.param<bool>("super_loam/useGpsElevation", useGpsElevation, false);
    // nh.param<float>("super_loam/gpsCovThreshold", gpsCovThreshold, 2.0);
    // nh.param<float>("super_loam/poseCovThreshold", poseCovThreshold, 25.0);

    // nh.param<bool>("super_loam/savePCD", savePCD, false);
    // nh.param<std::string>("super_loam/savePCDDirectory", savePCDDirectory,
    //                       "/Downloads/LOAM/");

    // nh.param<int>("super_loam/N_SCAN", N_SCAN, 16);
    // nh.param<int>("super_loam/Horizon_SCAN", Horizon_SCAN, 1800);

    // nh.param<float>("super_loam/imuAccNoise", imuAccNoise, 0.01);
    // nh.param<float>("super_loam/imuGyrNoise", imuGyrNoise, 0.001);
    // nh.param<float>("super_loam/imuAccBiasN", imuAccBiasN, 0.0002);
    // nh.param<float>("super_loam/imuGyrBiasN", imuGyrBiasN, 0.00003);
    // nh.param<float>("super_loam/imuGravity", imuGravity, 9.80511);
    // nh.param<vector<double>>("super_loam/extrinsicRot", extRotV, vector<double>());
    // nh.param<vector<double>>("super_loam/extrinsicRPY", extRPYV, vector<double>());
    // nh.param<vector<double>>("super_loam/extrinsicTrans", extTransV,
    //                          vector<double>());

    // nh.param<vector<double>>("super_loam/extrinsicRotation", extRimu_camV,
    //                          vector<double>());
    // nh.param<vector<double>>("super_loam/extrinsicTranslation", extTranimu_camV,
    //                          vector<double>());
    // nh.param<vector<double>>("super_loam/extrinsicRotation_camera_laser",
    //                          extRcam_laserV, vector<double>());
    // nh.param<vector<double>>("super_loam/extrinsicTranslation_camera_laser",
    //                          extTrancam_laserV, vector<double>());

    // extRimu_cam =
    //     Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
    //         extRimu_camV.data(), 3, 3);

    // extTranimu_cam =
    //     Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
    //         extTranimu_camV.data(), 3, 1);

    // extRcam_laser =
    //     Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
    //         extRcam_laserV.data(), 3, 3);

    // extTrancam_laser =
    //     Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
    //         extTrancam_laserV.data(), 3, 1);

    // extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
    //     extRotV.data(), 3, 3);
    // extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
    //     extRPYV.data(), 3, 3);
    // extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
    //     extTransV.data(), 3, 1);
    // extQRPY = Eigen::Quaterniond(extRPY);

    // nh.param<float>("super_loam/edgeThreshold", edgeThreshold, 0.1);
    // nh.param<float>("super_loam/surfThreshold", surfThreshold, 0.1);
    // nh.param<int>("super_loam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
    // nh.param<int>("super_loam/surfFeatureMinValidNum", surfFeatureMinValidNum,
    //               100);

    // nh.param<float>("super_loam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
    // nh.param<float>("super_loam/mappingCornerLeafSize", mappingCornerLeafSize,
    //                 0.2);
    // nh.param<float>("super_loam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

    // nh.param<float>("super_loam/z_tollerance", z_tollerance, FLT_MAX);
    // nh.param<float>("super_loam/rotation_tollerance", rotation_tollerance,
    //                 FLT_MAX);

    // nh.param<int>("super_loam/numberOfCores", numberOfCores, 2);
    // nh.param<double>("super_loam/mappingProcessInterval", mappingProcessInterval,
    //                  0.15);

    // nh.param<float>("super_loam/surroundingkeyframeAddingDistThreshold",
    //                 surroundingkeyframeAddingDistThreshold, 1.0);
    // nh.param<float>("super_loam/surroundingkeyframeAddingAngleThreshold",
    //                 surroundingkeyframeAddingAngleThreshold, 0.2);
    // nh.param<float>("super_loam/surroundingKeyframeDensity",
    //                 surroundingKeyframeDensity, 1.0);
    // nh.param<float>("super_loam/surroundingKeyframeSearchRadius",
    //                 surroundingKeyframeSearchRadius, 50.0);

    // nh.param<bool>("super_loam/loopClosureEnableFlag", loopClosureEnableFlag,
    //                false);
    // nh.param<int>("super_loam/surroundingKeyframeSize", surroundingKeyframeSize,
    //               50);
    // nh.param<float>("super_loam/historyKeyframeSearchRadius",
    //                 historyKeyframeSearchRadius, 10.0);
    // nh.param<float>("super_loam/historyKeyframeSearchTimeDiff",
    //                 historyKeyframeSearchTimeDiff, 30.0);
    // nh.param<int>("super_loam/historyKeyframeSearchNum", historyKeyframeSearchNum,
    //               25);
    // nh.param<float>("super_loam/historyKeyframeFitnessScore",
    //                 historyKeyframeFitnessScore, 0.3);

    // nh.param<float>("super_loam/globalMapVisualizationSearchRadius",
    //                 globalMapVisualizationSearchRadius, 1e3);
    // nh.param<float>("super_loam/globalMapVisualizationPoseDensity",
    //                 globalMapVisualizationPoseDensity, 10.0);
    // nh.param<float>("super_loam/globalMapVisualizationLeafSize",
    //                 globalMapVisualizationLeafSize, 1.0);

    usleep(100);
  }

  sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu &imu_in) {
    sensor_msgs::msg::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x,
                        imu_in.linear_acceleration.y,
                        imu_in.linear_acceleration.z);
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y,
                        imu_in.angular_velocity.z);

    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q(imu_in.orientation.w, imu_in.orientation.x,
                         imu_in.orientation.y, imu_in.orientation.z);

    imu_out.orientation.x = q.x();
    imu_out.orientation.y = q.y();
    imu_out.orientation.z = q.z();
    imu_out.orientation.w = q.w();

    return imu_out;
  }
};

sensor_msgs::msg::PointCloud2
publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<PointType>::Ptr thisCloud,
                                rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}


template <typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu::SharedPtr thisImuMsg, T *angular_x,
                           T *angular_y, T *angular_z) {
  *angular_x = thisImuMsg->angular_velocity.x;
  *angular_y = thisImuMsg->angular_velocity.y;
  *angular_z = thisImuMsg->angular_velocity.z;
}

template <typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu::SharedPtr thisImuMsg, T *acc_x, T *acc_y,
                       T *acc_z) {
  *acc_x = thisImuMsg->linear_acceleration.x;
  *acc_y = thisImuMsg->linear_acceleration.y;
  *acc_z = thisImuMsg->linear_acceleration.z;
}

template <typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu::SharedPtr thisImuMsg, T *rosRoll, T *rosPitch,
                   T *rosYaw) {
  double imuRoll, imuPitch, imuYaw;
  tf2::Quaternion orientation;
  tf2::fromMsg(thisImuMsg->orientation, orientation);
  tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

  *rosRoll = imuRoll;
  *rosPitch = imuPitch;
  *rosYaw = imuYaw;
}

 float pointDistance(PointType p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(PointType p1, PointType p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z));
}

#endif