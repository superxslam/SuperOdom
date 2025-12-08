#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
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
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("sensor_frame", "sensor");
    this->declare_parameter<std::string>("gravity_frame", "gravity");
    this->declare_parameter<std::string>("imu_odom_in", "/state_estimation");
    this->declare_parameter<std::string>("lidar_odom_in", "/laser_odometry");
    this->declare_parameter<std::string>("map_in", "/laser_cloud_map");
    this->declare_parameter<std::string>("scan_in", "/registered_scan");
    this->declare_parameter<std::string>("imu_odom_out", "/SuperOdom/state_estimation_aligned");
    this->declare_parameter<std::string>("lidar_odom_out", "/SuperOdom/laser_odometry_aligned");
    this->declare_parameter<std::string>("map_out", "/SuperOdom/laser_cloud_map_aligned");
    this->declare_parameter<std::string>("scan_out", "/SuperOdom/registered_scan_aligned");
    this->declare_parameter<bool>("rotate_twist", false);

    frame_map_ = this->get_parameter("map_frame").as_string();
    frame_sensor_ = this->get_parameter("sensor_frame").as_string();
    frame_gravity_ = this->get_parameter("gravity_frame").as_string();
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
    pub_imu_path_ = this->create_publisher<nav_msgs::msg::Path>("/SuperOdom/state_estimation_aligned_path", 10);
    pub_lio_path_ = this->create_publisher<nav_msgs::msg::Path>("/SuperOdom/laser_odometry_aligned_path", 10);

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
      auto ts = tf_buffer_->lookupTransform(frame_sensor_, frame_gravity_, tf2::TimePointZero);
      Eigen::Isometry3d T = tf2::transformToEigen(ts.transform);
      R_ = T.rotation();
      have_tf_ = true;
    } catch (const std::exception &e) {
      have_tf_ = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for TF %s -> %s: %s", frame_sensor_.c_str(), frame_gravity_.c_str(), e.what());
    }
  }
  
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg, bool is_imu) {

    Eigen::Matrix3d Rmsg;
    try {
      auto ts = tf_buffer_->lookupTransform(frame_sensor_, frame_gravity_, msg->header.stamp);
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
    out.header.frame_id = frame_map_;
    out.child_frame_id = frame_gravity_;
    if (is_imu) {
      pub_imu_->publish(out);
      updatePath(out, imu_path_, pub_imu_path_, true);
    } else {
      pub_lio_->publish(out);
      updatePath(out, lio_path_, pub_lio_path_, false);
    }
  }

  void updatePath(const nav_msgs::msg::Odometry &odom, nav_msgs::msg::Path &path, 
                  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub, bool is_imu) {
    static double last_imu_time = -1, last_lio_time = -1;
    double t = rclcpp::Time(odom.header.stamp).seconds();
    double &last_time = is_imu ? last_imu_time : last_lio_time;
    
    if (t - last_time > 0.1) {
      last_time = t;
      geometry_msgs::msg::PoseStamped ps;
      ps.header = odom.header;
      ps.pose = odom.pose.pose;
      path.poses.push_back(ps);
      while (path.poses.size() > 300) path.poses.erase(path.poses.begin());
      if (pub->get_subscription_count() > 0) {
        path.header = odom.header;
        pub->publish(path);
      }
    }
  }


  void mapCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    Eigen::Matrix3d Rmsg;
    try {
      auto ts = tf_buffer_->lookupTransform(frame_sensor_, frame_gravity_, msg->header.stamp);
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
    out.header.frame_id = frame_map_;
    pub_map_->publish(out);
  }

  void scanCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    Eigen::Matrix3d Rmsg;
    try {
      auto ts = tf_buffer_->lookupTransform(frame_sensor_, frame_gravity_, msg->header.stamp);
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
    out.header.frame_id = frame_map_;
    pub_scan_->publish(out);
  }

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  bool have_tf_ = false;

  // Topics
  std::string frame_map_, frame_sensor_, frame_gravity_;
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_imu_path_, pub_lio_path_;
  nav_msgs::msg::Path imu_path_, lio_path_;
  bool rotate_twist_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameNormalizer>());
  rclcpp::shutdown();
  return 0;
}


