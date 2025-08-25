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

    // Subscribers and publishers
    // Use compatible QoS: RELIABLE for LiDAR, BEST_EFFORT for IMU
    rclcpp::QoS lidar_qos(20);
    lidar_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    lidar_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    
    rclcpp::QoS imu_qos(10);
    imu_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    imu_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    
    // Subscribe to Livox CustomMsg only (avoids type conflicts)
    livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_in_, lidar_qos,
        std::bind(&SensorFlipNode::onLivoxCustom, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_in_, imu_qos,
        std::bind(&SensorFlipNode::onImu, this, std::placeholders::_1));

    livox_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(lidar_out_, lidar_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_out_, imu_qos);    RCLCPP_INFO(get_logger(), "sensor_flip configured: lidar %s -> %s, imu %s -> %s, RPY(deg)=(%.1f, %.1f, %.1f)",
                lidar_in_.c_str(), lidar_out_.c_str(), imu_in_.c_str(), imu_out_.c_str(),
                roll_deg, pitch_deg, yaw_deg);
  }

private:
  void onLivoxCustom(livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    if (!livox_pub_) return;
    auto out = *msg;
    // Flip xyz for each point; timestamp and metadata unchanged
    for (auto &pt : out.points) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      p = R_ * p;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
    }
    livox_pub_->publish(out);
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
    Eigen::Quaterniond qR(R_);
    Eigen::Quaterniond q_new = q * qR.conjugate();
    q_new.normalize();
    out.orientation.w = q_new.w();
    out.orientation.x = q_new.x();
    out.orientation.y = q_new.y();
    out.orientation.z = q_new.z();

    // Angular velocity and linear acceleration are vectors in IMU frame
    Eigen::Vector3d w(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Eigen::Vector3d a(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    w = R_ * w;  a = R_ * a;
    out.angular_velocity.x = w.x(); out.angular_velocity.y = w.y(); out.angular_velocity.z = w.z();
    out.linear_acceleration.x = a.x(); out.linear_acceleration.y = a.y(); out.linear_acceleration.z = a.z();

    auto rotCov = [&](std::array<double, 9>& P){
      Eigen::Map<Eigen::Matrix3d> M(P.data());
      if (M(0,0) >= 0.0) { M = R_ * M * R_.transpose(); }
    };

    out.header.frame_id = msg->header.frame_id;
    imu_pub_->publish(out);
  }

  std::string lidar_in_, imu_in_, lidar_out_, imu_out_;
  Eigen::Matrix3d R_;
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
