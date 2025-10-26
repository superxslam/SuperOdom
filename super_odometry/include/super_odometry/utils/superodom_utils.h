// Created by Shibo Zhao on 2025-03-31

# pragma once
#ifndef SUPER_ODOMETRY_LASER_MAPPING_UTILS_H
#define SUPER_ODOMETRY_LASER_MAPPING_UTILS_H

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <string>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "super_odometry/utils/Twist.h"
#include <queue>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "super_odometry/sensor_data/pointcloud/point_os.h"


namespace super_odometry {
namespace utils {


class ScopedTimer {
public:
    explicit ScopedTimer(const std::string& name, rclcpp::Logger logger = rclcpp::get_logger("ScopedTimer"))
        : name_(name), 
          start_(std::chrono::high_resolution_clock::now()),
          logger_(logger) {}

    ~ScopedTimer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_).count();
        RCLCPP_DEBUG(logger_, "%s took %ld ms", name_.c_str(), duration);
    }

private:
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
    rclcpp::Logger logger_;
};


struct OdometryData {
    double timestamp;
    double duration;
    double x, y, z;
    double roll, pitch, yaw;
};

extern std::vector<OdometryData> odometryResults;

void transformAssociateToMap(Transformd& T_w_curr, 
                           const Transformd& T_w_pre, 
                           const Transformd& T_wodom_curr, 
                           const Transformd& T_wodom_pre);

inline void transformUpdate(const Eigen::Quaterniond &q_w_curr, const Eigen::Vector3d &t_w_curr,
                           const Eigen::Quaterniond &q_wodom_curr, const Eigen::Vector3d &t_wodom_curr,
                           Eigen::Quaterniond &q_wmap_wodom, Eigen::Vector3d &t_wmap_wodom) {
    q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
    t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}


bool readPointCloud(const std::string &file_path, pcl::PointCloud<PointType>::Ptr cloud_out);


bool readLocalizationPose(const std::string &file_path, std::vector<OdometryData> &odometry_results);


bool saveLocalizationPose(double timestamp, const Transformd &T_w_lidar, 
                         const std::string &file_path, std::vector<OdometryData> &odometry_results);

// Transform utilities
void transformAssociateToMap(Transformd& T_w_curr, 
                           const Transformd& T_w_pre, 
                           const Transformd& T_wodom_curr, 
                           const Transformd& T_wodom_pre);

void transformAssociateToMap(Eigen::Quaterniond& q_w_curr,
                           Eigen::Vector3d& t_w_curr,
                           const Eigen::Quaterniond& q_wmap_wodom,
                           const Eigen::Quaterniond& q_wodom_curr,
                           const Eigen::Vector3d& t_wodom_curr,
                           const Eigen::Vector3d& t_wmap_wodom);

void transformUpdate(Eigen::Quaterniond& q_wmap_wodom,
                    Eigen::Vector3d& t_wmap_wodom,
                    const Eigen::Quaterniond& q_w_curr,
                    const Eigen::Quaterniond& q_wodom_curr,
                    const Eigen::Vector3d& t_w_curr,
                    const Eigen::Vector3d& t_wodom_curr);

void pointAssociateToMap(PointType const *const pi, PointType *const po,
                        const Eigen::Quaterniond& q_w_curr,
                        const Eigen::Vector3d& t_w_curr);

void pointAssociateToMap(pcl::PointXYZHSV const *const pi, pcl::PointXYZHSV *const po,
                        const Eigen::Quaterniond& q_w_curr,
                        const Eigen::Vector3d& t_w_curr);

void pointAssociateTobeMapped(PointType const *const pi, PointType *const po,
                            const Eigen::Quaterniond& q_w_curr,
                            const Eigen::Vector3d& t_w_curr);

tf2::Quaternion extractRollPitch(Eigen::Quaterniond& imu_rotation);

void printTransform(const Transformd& T, const std::string& name);

void transformOusterPoints(point_os::OusterPointXYZIRT const *const pi, point_os::PointcloudXYZITR *const po, Transformd &transform);

bool savePly(pcl::PointCloud<PointType>::Ptr pcl_to_save, rclcpp::Node::SharedPtr node);


template<typename T>
inline constexpr T Deg2Rad(const T &deg) { return deg / 180. * M_PI; }

template<typename PointT>
inline void TransformPoint(PointT &p, const Transformd &transform) {
    Eigen::Vector3d temp = p.getVector3fMap().template cast<double>();
    p.getVector3fMap() = (transform * temp).template cast<float>();
}

template<typename PointT>
inline PointT TransformPointd(const PointT &p, const Transformd &transform) {
    PointT out(p);
    TransformPoint(out, transform);
    return out;
}

inline bool compare_pair_first(const std::pair<float, int> a, const std::pair<float, int> b) // sort from big to small
{
    return a.first > b.first;
}

/*!
* @brief Compute PCA of Nx3 data array and mean value
* @param[in] data Nx3 array (e.g. stacked 3D points)
* @param[out] mean Where to store mean value
* @return The PCA
*/
inline Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
ComputePCA(const Eigen::Matrix<double, Eigen::Dynamic, 3> &data,
            Eigen::Vector3d &mean) {
    mean = data.colwise().mean();
    Eigen::MatrixXd centered = data.rowwise() - mean.transpose();
    Eigen::Matrix3d varianceCovariance = centered.transpose() * centered;

    return Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(varianceCovariance);
}

//------------------------------------------------------------------------------
/*!
* @brief Compute PCA of Nx3 data array and mean value
* @param data Nx3 array (e.g. stacked 3D points)
* @return The PCA
*/
inline Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
ComputePCA(const Eigen::Matrix<double, Eigen::Dynamic, 3> &data) {
    Eigen::Vector3d mean;
    return ComputePCA(data, mean);
}

template<typename T>
inline constexpr T Rad2Deg(const T &rad) { return rad / M_PI * 180.; }

} // namespace utils
} // namespace super_odometry

#endif // SUPER_ODOMETRY_LASER_MAPPING_UTILS_H
