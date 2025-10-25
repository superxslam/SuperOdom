#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

class FrameNormalizer : public rclcpp::Node {
public:
  FrameNormalizer() : Node("frame_normalizer") {
    this->declare_parameter<std::string>("map_aligned", "map");
    this->declare_parameter<std::string>("gravity", "gravity_aligned");
    this->declare_parameter<std::string>("imu_odom_in", "/state_estimation");
    this->declare_parameter<std::string>("lidar_odom_in", "/laser_odometry");
    this->declare_parameter<std::string>("map_in", "/laser_cloud_map");
    this->declare_parameter<std::string>("scan_in", "/registered_scan");
    this->declare_parameter<std::string>("imu_odom_out", "/SuperOdom/state_estimation_aligned");
    this->declare_parameter<std::string>("lidar_odom_out", "/SuperOdom/laser_odometry_aligned");
    this->declare_parameter<std::string>("map_out", "/SuperOdom/laser_cloud_map_aligned");
    this->declare_parameter<std::string>("scan_out", "/SuperOdom/registered_scan_aligned");
    this->declare_parameter<bool>("rotate_twist", false);

    frame_aligned_ = this->get_parameter("map_aligned").as_string();
    frame_raw_ = this->get_parameter("gravity").as_string();
    imu_in_ = this->get_parameter("imu_odom_in").as_string();
    lio_in_ = this->get_parameter("lidar_odom_in").as_string();
    map_in_ = this->get_parameter("map_in").as_string();
    scan_in_ = this->get_parameter("scan_in").as_string();
    imu_out_ = this->get_parameter("imu_odom_out").as_string();
    lio_out_ = this->get_parameter("lidar_odom_out").as_string();
    map_out_ = this->get_parameter("map_out").as_string();
    scan_out_ = this->get_parameter("scan_out").as_string();
    rotate_twist_ = this->get_parameter("rotate_twist").as_bool();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_imu_ = this->create_publisher<nav_msgs::msg::Odometry>(imu_out_, 10);
    pub_lio_ = this->create_publisher<nav_msgs::msg::Odometry>(lio_out_, 10);
    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_out_, 2);
    pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(scan_out_, 2);

    sub_imu_ = this->create_subscription<nav_msgs::msg::Odometry>(
      imu_in_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->odomCb(msg, true); });
    sub_lio_ = this->create_subscription<nav_msgs::msg::Odometry>(
      lio_in_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->odomCb(msg, false); });
    sub_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(map_in_, 2,
      std::bind(&FrameNormalizer::mapCb, this, std::placeholders::_1));
    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(scan_in_, 5,
      std::bind(&FrameNormalizer::scanCb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&FrameNormalizer::refreshTransform, this));
  }

private:
  void refreshTransform() {
    try {
      auto ts = tf_buffer_->lookupTransform(frame_aligned_, frame_raw_, tf2::TimePointZero);
      Eigen::Isometry3d T = tf2::transformToEigen(ts.transform);
      R_ = T.rotation();
      have_tf_ = true;
    } catch (const std::exception &e) {
      have_tf_ = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for TF %s -> %s: %s", frame_aligned_.c_str(), frame_raw_.c_str(), e.what());
    }
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg, bool is_imu) {

    Eigen::Matrix3d Rmsg;
    try {
      auto ts = tf_buffer_->lookupTransform(frame_aligned_, frame_raw_, msg->header.stamp);
      Eigen::Isometry3d T = tf2::transformToEigen(ts.transform);
      Rmsg = T.rotation();
    } catch (const std::exception &e) {
      // Fallback to latest if stamped lookup fails
      if (!have_tf_) return;
      Rmsg = R_;
    }
    nav_msgs::msg::Odometry out = *msg;

    // Orientation
    Eigen::Quaterniond q_in(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);
    Eigen::Quaterniond q_out = Eigen::Quaterniond(Rmsg) * q_in;
    q_out.normalize();
    out.pose.pose.orientation.w = q_out.w();
    out.pose.pose.orientation.x = q_out.x();
    out.pose.pose.orientation.y = q_out.y();
    out.pose.pose.orientation.z = q_out.z();

    // Position
    Eigen::Vector3d t_in(msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.position.z);
    Eigen::Vector3d t_out = Rmsg * t_in;
    out.pose.pose.position.x = t_out.x();
    out.pose.pose.position.y = t_out.y();
    out.pose.pose.position.z = t_out.z();

    // Twists
    Eigen::Vector3d v_in(msg->twist.twist.linear.x,
                         msg->twist.twist.linear.y,
                         msg->twist.twist.linear.z);
    Eigen::Vector3d w_in(msg->twist.twist.angular.x,
                         msg->twist.twist.angular.y,
                         msg->twist.twist.angular.z);
    if (rotate_twist_) {
      Eigen::Vector3d v_out = Rmsg * v_in;
      Eigen::Vector3d w_out = Rmsg * w_in;
      out.twist.twist.linear.x = v_out.x();
      out.twist.twist.linear.y = v_out.y();
      out.twist.twist.linear.z = v_out.z();
      out.twist.twist.angular.x = w_out.x();
      out.twist.twist.angular.y = w_out.y();
      out.twist.twist.angular.z = w_out.z();
    }

    // Frame id to aligned
    out.header.frame_id = frame_aligned_;
    if (is_imu) pub_imu_->publish(out);
    else        pub_lio_->publish(out);
  }

  void mapCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    Eigen::Matrix3d Rmsg;
    try {
      auto ts = tf_buffer_->lookupTransform(frame_aligned_, frame_raw_, msg->header.stamp);
      Eigen::Isometry3d T = tf2::transformToEigen(ts.transform);
      Rmsg = T.rotation();
    } catch (const std::exception &e) {
      if (!have_tf_) return;
      Rmsg = R_;
    }
    pcl::PointCloud<pcl::PointXYZI> cloud_in, cloud_out;
    pcl::fromROSMsg(*msg, cloud_in);
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = Rmsg.cast<float>();
    pcl::transformPointCloud(cloud_in, cloud_out, T);
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(cloud_out, out);
    out.header = msg->header;
    out.header.frame_id = frame_aligned_;
    pub_map_->publish(out);
  }

  void scanCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    Eigen::Matrix3d Rmsg;
    try {
      auto ts = tf_buffer_->lookupTransform(frame_aligned_, frame_raw_, msg->header.stamp);
      Eigen::Isometry3d T = tf2::transformToEigen(ts.transform);
      Rmsg = T.rotation();
    } catch (const std::exception &e) {
      if (!have_tf_) return;
      Rmsg = R_;
    }
    pcl::PointCloud<pcl::PointXYZI> cloud_in, cloud_out;
    pcl::fromROSMsg(*msg, cloud_in);
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = Rmsg.cast<float>();
    pcl::transformPointCloud(cloud_in, cloud_out, T);
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(cloud_out, out);
    out.header = msg->header;
    out.header.frame_id = frame_aligned_;
    pub_scan_->publish(out);
  }

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  bool have_tf_ = false;

  // Topics
  std::string frame_aligned_, frame_raw_;
  std::string imu_in_, lio_in_, map_in_;
  std::string scan_in_;
  std::string imu_out_, lio_out_, map_out_, scan_out_;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_imu_, sub_lio_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_, pub_lio_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
  bool rotate_twist_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameNormalizer>());
  rclcpp::shutdown();
  return 0;
}


