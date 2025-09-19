// Created by Shibo on 2025-03-29.
//

#include "super_odometry/utils/superodom_utils.h"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl/io/ply_io.h>
#include <filesystem>

namespace super_odometry {
namespace utils {

std::vector<OdometryData> odometryResults;


bool readPointCloud(const std::string &file_path, pcl::PointCloud<PointType>::Ptr cloud_out) {
    std::ifstream file_check(file_path.c_str());
    if (!file_check.good()) {
        std::cerr << "Error: File does not exist: " << file_path << std::endl;
        return false;
    }
    file_check.close();
    
    pcl::PCDReader reader;
    int result = reader.read(file_path, *cloud_out);
    
    if (result < 0) {
        std::cerr << "Error reading PCD file: " << file_path << std::endl;
        return false;
    }
    
    return true;
}

bool readLocalizationPose(const std::string &file_path, std::vector<OdometryData> &odometry_results) {
    std::string localizationPosePath = file_path;
    
    // If file_path is a directory, append "start_pose.txt"
    size_t lastSlashPos = file_path.find_last_of('/');
    if (lastSlashPos != std::string::npos) {
        std::string directory = file_path.substr(0, lastSlashPos + 1);
        localizationPosePath = directory + "start_pose.txt";
    }
    
    std::ifstream file(localizationPosePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << localizationPosePath << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        
        std::istringstream iss(line);
        OdometryData odom;
        if (iss >> odom.duration >> odom.x >> odom.y >> odom.z >> odom.roll >> odom.pitch >> odom.yaw) {
            std::cout << "Read odometry data: " << odom.x << " " << odom.y << " " << odom.z << std::endl;
            odometry_results.push_back(odom);
        } else {
            std::cerr << "Error reading line: " << line << std::endl;
        }
    }
    
    if (!odometry_results.empty()) {
        std::cout << "\033[1;32m Loaded the localization_pose.txt successfully \033[0m" 
                  << odometry_results[0].x << " " << odometry_results[0].y << " " 
                  << odometry_results[0].z << std::endl;
    }
    
    file.close();
    return !odometry_results.empty();
}

bool saveLocalizationPose(double timestamp, const Transformd &T_w_lidar, 
                         const std::string &file_path, std::vector<OdometryData> &odometry_results) {
    std::string saveOdomPath;
    size_t lastSlashPos = file_path.find_last_of('/');
    if (lastSlashPos != std::string::npos) {
        saveOdomPath = file_path.substr(0, lastSlashPos + 1); // Include the trailing slash
    } else {
        saveOdomPath = "./";
    }

    OdometryData odom;
    {
        odom.timestamp = timestamp;
        odom.x = T_w_lidar.pos.x();
        odom.y = T_w_lidar.pos.y();
        odom.z = T_w_lidar.pos.z(); 
        tf2::Quaternion orientation(T_w_lidar.rot.x(), T_w_lidar.rot.y(), T_w_lidar.rot.z(), T_w_lidar.rot.w());
        tf2::Matrix3x3(orientation).getRPY(odom.roll, odom.pitch, odom.yaw);
    }

    odometry_results.push_back(odom);
    
    std::string OdomResultPath = saveOdomPath + "start_pose.txt";
    std::ofstream outFile(OdomResultPath, std::ios::app);
    
    if (!outFile.is_open()) {
        std::cerr << "Error opening file: " << OdomResultPath << std::endl;
        return false;
    }

    outFile << std::fixed << (odometry_results.size() > 1 ? (odom.timestamp - odometry_results[0].timestamp) : 0.0) << " "
            << odom.x << " " << odom.y << " " << odom.z << " "
            << odom.roll << " " << odom.pitch << " " << odom.yaw << std::endl;

    outFile.close();
    return true;
}

void transformAssociateToMap(Transformd& T_w_curr, 
                           const Transformd& T_w_pre, 
                           const Transformd& T_wodom_curr, 
                           const Transformd& T_wodom_pre) {
    // Calculate relative transform between previous and current odometry
    Transformd T_wodom_pre_curr = T_wodom_pre.inverse() * T_wodom_curr;
    
    // Apply the relative transform to the previous world pose
    T_w_curr = T_w_pre * T_wodom_pre_curr;
}

void transformAssociateToMap(Eigen::Quaterniond& q_w_curr,
                           Eigen::Vector3d& t_w_curr,
                           const Eigen::Quaterniond& q_wmap_wodom,
                           const Eigen::Quaterniond& q_wodom_curr,
                           const Eigen::Vector3d& t_wodom_curr,
                           const Eigen::Vector3d& t_wmap_wodom) {
    // Transform from odometry frame to world frame
    q_w_curr = q_wmap_wodom * q_wodom_curr;
    t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

void transformUpdate(Eigen::Quaterniond& q_wmap_wodom,
                    Eigen::Vector3d& t_wmap_wodom,
                    const Eigen::Quaterniond& q_w_curr,
                    const Eigen::Quaterniond& q_wodom_curr,
                    const Eigen::Vector3d& t_w_curr,
                    const Eigen::Vector3d& t_wodom_curr) {
    // Update the transform between world and odometry frames
    q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
    t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}

void pointAssociateToMap(PointType const *const pi, PointType *const po,
                        const Eigen::Quaterniond& q_w_curr,
                        const Eigen::Vector3d& t_w_curr) {
    // Transform point from current frame to world frame
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
}

void pointAssociateToMap(pcl::PointXYZHSV const *const pi, pcl::PointXYZHSV *const po,
                        const Eigen::Quaterniond& q_w_curr,
                        const Eigen::Vector3d& t_w_curr) {
    // Transform HSV point from current frame to world frame
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->h = pi->h;
    po->s = pi->s;
    po->v = pi->v;
}

void pointAssociateTobeMapped(PointType const *const pi, PointType *const po,
                            const Eigen::Quaterniond& q_w_curr,
                            const Eigen::Vector3d& t_w_curr) {
    // Transform point from world frame to current frame
    Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
    po->x = point_curr.x();
    po->y = point_curr.y();
    po->z = point_curr.z();
    po->intensity = pi->intensity;
}


tf2::Quaternion extractRollPitch(Eigen::Quaterniond& imu_rotation){
    double imu_roll, imu_pitch, imu_yaw;
    tf2::Quaternion orientation(imu_rotation.x(), imu_rotation.y(), imu_rotation.z(), imu_rotation.w());
    tf2::Matrix3x3(orientation).getRPY(imu_roll, imu_pitch, imu_yaw);
    tf2::Quaternion quat ;
    quat.setRPY(imu_roll,imu_pitch, 0.0);
    RCLCPP_INFO(rclcpp::get_logger("super_odometry"), "Using IMU Roll Pitch in ICP: %f %f %f", imu_roll, imu_pitch, imu_yaw);
    return quat;
}

void printTransform(const Transformd& T, const std::string& name){
    std::cout<<name<<": "<<T.pos.transpose()<<std::endl;
    std::cout<<name<<": "<<T.rot<<std::endl;
}

void transformOusterPoints(point_os::OusterPointXYZIRT const *const pi, point_os::PointcloudXYZITR *const po, Transformd &transform) {
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = transform.rot * point_curr + transform.pos;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
}

bool savePly(pcl::PointCloud<PointType>::Ptr pcl_to_save, rclcpp::Node::SharedPtr node) {
    if (pcl_to_save->size() >= 10 && pcl_to_save->size() % 10 == 0) {
        std::string las_dir = std::string(ROOT_DIR) + "PLY";
        std::filesystem::create_directories(las_dir);
        std::string all_points_dir = las_dir + "/saved_scans.ply";
        if (pcl::io::savePLYFileBinary(all_points_dir, *pcl_to_save) == 0) {
            RCLCPP_INFO(node->get_logger(), "All scans saved to %s with %zu points", all_points_dir.c_str(), pcl_to_save->size());
            return true;
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to save scans to %s", all_points_dir.c_str());
            return false;
        }
    }
    return false;
}


} // namespace utils
} // namespace super_odometry
