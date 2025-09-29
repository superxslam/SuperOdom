#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>
#include <execution>
#include <algorithm>

enum class RotationType { IDENTITY, FLIP_X, FLIP_Y, FLIP_Z, FLIP_XY, GENERAL };

class SensorFlipNode : public rclcpp::Node {
public:
  SensorFlipNode() : Node("sensor_flip_node") {
    // Parameters
    lidar_in_ = declare_parameter<std::string>("lidar_in", "/livox/lidar");
    imu_in_ = declare_parameter<std::string>("imu_in", "/livox/imu");
    lidar_out_ = declare_parameter<std::string>("lidar_out", "/livox_flipped/lidar");
    imu_out_ = declare_parameter<std::string>("imu_out", "/livox_flipped/imu");
    double roll_deg = declare_parameter<double>("roll_deg", 180.0);
    double pitch_deg = declare_parameter<double>("pitch_deg", 0.0);
    double yaw_deg = declare_parameter<double>("yaw_deg", 0.0);

    // Build rotation matrix R from roll,pitch,yaw degrees
    const double pi = 3.14159265358979323846;
    double r = roll_deg * pi / 180.0;
    double p = pitch_deg * pi / 180.0;
    double y = yaw_deg * pi / 180.0;

    Eigen::AngleAxisd Rx(r, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Ry(p, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rz(y, Eigen::Vector3d::UnitZ());
    // ZYX (yaw-pitch-roll) composition commonly used
    R_ = (Rz * Ry * Rx).toRotationMatrix();

    // Pre-compute matrix elements and quaternion for optimization
    precomputeRotationElements();

    // Subscribers and publishers
    // Use compatible QoS: RELIABLE for LiDAR, BEST_EFFORT for IMU
    rclcpp::QoS lidar_qos(20);
    lidar_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    lidar_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    
    rclcpp::QoS imu_qos(10);
    imu_qos.best_effort();  // Use BEST_EFFORT reliability to match imu_preintegration
    imu_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    
    // Subscribe to Livox CustomMsg only (avoids type conflicts)
    livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_in_, lidar_qos,
        std::bind(&SensorFlipNode::onLivoxCustom, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_in_, imu_qos,
        std::bind(&SensorFlipNode::onImu, this, std::placeholders::_1));

    livox_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(lidar_out_, lidar_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_out_, imu_qos);
    
    RCLCPP_INFO(get_logger(), "sensor_flip configured: lidar %s -> %s, imu %s -> %s, RPY(deg)=(%.1f, %.1f, %.1f)",
                lidar_in_.c_str(), lidar_out_.c_str(), imu_in_.c_str(), imu_out_.c_str(),
                roll_deg, pitch_deg, yaw_deg);
  }

private:
  void precomputeRotationElements() {
    r00_ = R_(0,0); r01_ = R_(0,1); r02_ = R_(0,2);
    r10_ = R_(1,0); r11_ = R_(1,1); r12_ = R_(1,2);
    r20_ = R_(2,0); r21_ = R_(2,1); r22_ = R_(2,2);
    R_quat_ = Eigen::Quaterniond(R_);
    R_quat_conj_ = R_quat_.conjugate(); // Pre-compute conjugate
    
    // Detect rotation type for fast paths
    detectRotationType();
  }
  
  void detectRotationType() {
    const double eps = 1e-6;
    if (std::abs(r00_ + 1.0) < eps && std::abs(r11_ + 1.0) < eps && std::abs(r22_ - 1.0) < eps &&
        std::abs(r01_) < eps && std::abs(r02_) < eps && std::abs(r10_) < eps && 
        std::abs(r12_) < eps && std::abs(r20_) < eps && std::abs(r21_) < eps) {
      rotation_type_ = RotationType::FLIP_XY;
    } else if (std::abs(r00_ + 1.0) < eps && std::abs(r11_ - 1.0) < eps && std::abs(r22_ - 1.0) < eps &&
               std::abs(r01_) < eps && std::abs(r02_) < eps && std::abs(r10_) < eps && 
               std::abs(r12_) < eps && std::abs(r20_) < eps && std::abs(r21_) < eps) {
      rotation_type_ = RotationType::FLIP_X;
    } else if (std::abs(r00_ - 1.0) < eps && std::abs(r11_ + 1.0) < eps && std::abs(r22_ - 1.0) < eps &&
               std::abs(r01_) < eps && std::abs(r02_) < eps && std::abs(r10_) < eps && 
               std::abs(r12_) < eps && std::abs(r20_) < eps && std::abs(r21_) < eps) {
      rotation_type_ = RotationType::FLIP_Y;
    } else if (std::abs(r00_ - 1.0) < eps && std::abs(r11_ - 1.0) < eps && std::abs(r22_ + 1.0) < eps &&
               std::abs(r01_) < eps && std::abs(r02_) < eps && std::abs(r10_) < eps && 
               std::abs(r12_) < eps && std::abs(r20_) < eps && std::abs(r21_) < eps) {
      rotation_type_ = RotationType::FLIP_Z;
    } else if (std::abs(r00_ - 1.0) < eps && std::abs(r11_ - 1.0) < eps && std::abs(r22_ - 1.0) < eps &&
               std::abs(r01_) < eps && std::abs(r02_) < eps && std::abs(r10_) < eps && 
               std::abs(r12_) < eps && std::abs(r20_) < eps && std::abs(r21_) < eps) {
      rotation_type_ = RotationType::IDENTITY;
    } else {
      rotation_type_ = RotationType::GENERAL;
    }
  }
  
  void transformPointFast(livox_ros_driver2::msg::CustomPoint &pt) {
    switch (rotation_type_) {
      case RotationType::IDENTITY:
        break; // No transformation needed
      case RotationType::FLIP_X:
        pt.x = -pt.x; break;
      case RotationType::FLIP_Y:
        pt.y = -pt.y; break;
      case RotationType::FLIP_Z:
        pt.z = -pt.z; break;
      case RotationType::FLIP_XY:
        pt.x = -pt.x; pt.y = -pt.y; break;
      default: // GENERAL
        float x = pt.x, y = pt.y, z = pt.z;
        pt.x = r00_ * x + r01_ * y + r02_ * z;
        pt.y = r10_ * x + r11_ * y + r12_ * z;
        pt.z = r20_ * x + r21_ * y + r22_ * z;
    }
  }

  void onLivoxCustom(livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    // Check if we should process this message
    if (!livox_pub_ || livox_pub_->get_subscription_count() == 0) return;
    
    // Modify in-place if this is the only subscriber, otherwise copy
    auto out_msg = (livox_pub_->get_subscription_count() == 1) ? msg : 
                   std::make_shared<livox_ros_driver2::msg::CustomMsg>(*msg);
    
    // Use parallel processing for large point clouds
    if (out_msg->points.size() > 1000) {
      std::for_each(std::execution::par_unseq, out_msg->points.begin(), out_msg->points.end(),
        [this](livox_ros_driver2::msg::CustomPoint &pt) {
          transformPointFast(pt);
        });
    } else {
      // Serial processing with memory prefetching for smaller point clouds
      const size_t prefetch_distance = 64;
      for (size_t i = 0; i < out_msg->points.size(); ++i) {
        // Prefetch next cache line
        if (i + prefetch_distance < out_msg->points.size()) {
          __builtin_prefetch(&out_msg->points[i + prefetch_distance], 1, 3);
        }
        transformPointFast(out_msg->points[i]);
      }
    }
    livox_pub_->publish(*out_msg);
  }


  void onImu(sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!imu_pub_) return;
    sensor_msgs::msg::Imu out = *msg;

    // Rotate orientation, angular velocity, and linear acceleration
    // Convert quaternion to rotation matrix
    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    if (!(std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()))) {
      q = Eigen::Quaterniond::Identity();
    } else {
      q.normalize();
    }
    Eigen::Quaterniond q_new = q * R_quat_conj_; // Use pre-computed conjugate
    q_new.normalize();
    out.orientation.w = q_new.w();
    out.orientation.x = q_new.x();
    out.orientation.y = q_new.y();
    out.orientation.z = q_new.z();

    // Angular velocity and linear acceleration - direct computation for better performance
    double wx = msg->angular_velocity.x, wy = msg->angular_velocity.y, wz = msg->angular_velocity.z;
    double ax = msg->linear_acceleration.x, ay = msg->linear_acceleration.y, az = msg->linear_acceleration.z;
    
    out.angular_velocity.x = r00_ * wx + r01_ * wy + r02_ * wz;
    out.angular_velocity.y = r10_ * wx + r11_ * wy + r12_ * wz;
    out.angular_velocity.z = r20_ * wx + r21_ * wy + r22_ * wz;
    
    out.linear_acceleration.x = r00_ * ax + r01_ * ay + r02_ * az;
    out.linear_acceleration.y = r10_ * ax + r11_ * ay + r12_ * az;
    out.linear_acceleration.z = r20_ * ax + r21_ * ay + r22_ * az;
    
    auto rotCov = [&](std::array<double, 9>& P){
      Eigen::Map<Eigen::Matrix3d> M(P.data());
      if (M(0,0) >= 0.0) { M = R_ * M * R_.transpose(); }
    };

    out.header.frame_id = msg->header.frame_id;
    imu_pub_->publish(out);
  }

  std::string lidar_in_, imu_in_, lidar_out_, imu_out_;
  Eigen::Matrix3d R_;
  
  // Pre-computed matrix elements for faster point transformation
  double r00_, r01_, r02_, r10_, r11_, r12_, r20_, r21_, r22_;
  Eigen::Quaterniond R_quat_; // Pre-computed rotation as quaternion
  Eigen::Quaterniond R_quat_conj_; // Pre-computed conjugate
  RotationType rotation_type_; // Detected rotation type for fast paths
  
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorFlipNode>());
  rclcpp::shutdown();
  return 0;
}
