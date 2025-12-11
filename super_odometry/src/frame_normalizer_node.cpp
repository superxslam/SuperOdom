#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <mutex>
#include <deque>
#include <limits>
#include <sstream>

/**
 * @brief Node for gravity-aligning odometry/point clouds and colorizing scans with camera images
 *
 * This node performs two main functions:
 * 1. Frame alignment: Transforms odometry and point clouds to a gravity-aligned reference frame
 * 2. Point cloud colorization: Projects lidar points onto synchronized camera images with
 *    bilinear interpolation for high-quality RGB coloring
 */
class FrameNormalizer : public rclcpp::Node {
public:
  FrameNormalizer() : Node("frame_normalizer") {
    RCLCPP_INFO(this->get_logger(), "=== FrameNormalizer Constructor Started ===");
    
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

    // Colorization parameters
    this->declare_parameter<bool>("use_compressed_image", false);
    this->declare_parameter<std::string>("image_in", "/your/color/image");
    this->declare_parameter<std::string>("camera_info_in", "/your/color/camera_info");
    this->declare_parameter<std::string>("camera_frame", "camera_frame");
    this->declare_parameter<std::string>("scan_color_out", "/SuperOdom/registered_scan_color");
    this->declare_parameter<double>("sync_tolerance", 0.05);

    // Camera extrinsics (sensor -> camera)
    this->declare_parameter<bool>("use_static_extrinsics", true);
    this->declare_parameter<double>("cam_tx", -0.010997);
    this->declare_parameter<double>("cam_ty", -0.038107);
    this->declare_parameter<double>("cam_tz", -0.020099);
    this->declare_parameter<double>("cam_roll_deg", 0.0);
    this->declare_parameter<double>("cam_pitch_deg", 0.0);
    this->declare_parameter<double>("cam_yaw_deg", 0.0);
    this->declare_parameter<double>("cam_qx", -0.473078);
    this->declare_parameter<double>("cam_qy", -0.473175);
    this->declare_parameter<double>("cam_qz", -0.525554);
    this->declare_parameter<double>("cam_qw", 0.525447);

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
    
    // Debug: Check parameter before reading
    RCLCPP_INFO(this->get_logger(), "=== Reading use_compressed_image parameter ===");
    if (this->has_parameter("use_compressed_image")) {
      RCLCPP_INFO(this->get_logger(), "Parameter 'use_compressed_image' exists");
      auto param = this->get_parameter("use_compressed_image");
      RCLCPP_INFO(this->get_logger(), "Parameter type: %s", param.get_type_name().c_str());
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        RCLCPP_INFO(this->get_logger(), "Parameter value (as_bool): %s", param.as_bool() ? "true" : "false");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'use_compressed_image' NOT FOUND!");
    }
    
    use_compressed_image_ = this->get_parameter("use_compressed_image").as_bool();
    image_in_ = this->get_parameter("image_in").as_string();
    cam_info_in_ = this->get_parameter("camera_info_in").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    scan_color_out_ = this->get_parameter("scan_color_out").as_string();
    sync_tolerance_ = this->get_parameter("sync_tolerance").as_double();
    
    // Debug: Print all loaded parameters
    RCLCPP_INFO(this->get_logger(), "=== Frame Normalizer Parameters ===");
    RCLCPP_INFO(this->get_logger(), "use_compressed_image: %s (value: %d)", 
      use_compressed_image_ ? "true" : "false", static_cast<int>(use_compressed_image_));
    RCLCPP_INFO(this->get_logger(), "image_in: %s", image_in_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_info_in: %s", cam_info_in_.c_str());
    RCLCPP_INFO(this->get_logger(), "scan_color_out: %s", scan_color_out_.c_str());
    use_static_extrinsics_ = this->get_parameter("use_static_extrinsics").as_bool();
    if (use_static_extrinsics_) {
      const double tx = this->get_parameter("cam_tx").as_double();
      const double ty = this->get_parameter("cam_ty").as_double();
      const double tz = this->get_parameter("cam_tz").as_double();
      // Prefer quaternion if provided (norm >= ~0.5)
      const double qx = this->get_parameter("cam_qx").as_double();
      const double qy = this->get_parameter("cam_qy").as_double();
      const double qz = this->get_parameter("cam_qz").as_double();
      const double qw = this->get_parameter("cam_qw").as_double();
      const double qnorm = std::sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
      
      Eigen::Matrix3d R_camera_sensor;  // From YAML: camera -> sensor (lidar)
      Eigen::Vector3d t_camera_sensor(tx, ty, tz);
      
      if (qnorm > 0.5) {
        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();
        R_camera_sensor = q.toRotationMatrix();
      } else {
        const double r = this->get_parameter("cam_roll_deg").as_double() * M_PI / 180.0;
        const double p = this->get_parameter("cam_pitch_deg").as_double() * M_PI / 180.0;
        const double y = this->get_parameter("cam_yaw_deg").as_double() * M_PI / 180.0;
        Eigen::AngleAxisd Rx(r, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(p, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(y, Eigen::Vector3d::UnitZ());
        R_camera_sensor = (Rz * Ry * Rx).toRotationMatrix();
      }
      
      // Invert transform: T_sensor_camera = T_camera_sensor^-1
      // R_sensor_camera = R_camera_sensor^T
      // t_sensor_camera = -R_camera_sensor^T * t_camera_sensor
      R_sc_static_ = R_camera_sensor.transpose();
      t_sc_static_ = -R_sc_static_ * t_camera_sensor;
      
      RCLCPP_INFO(this->get_logger(), "Loaded camera->sensor extrinsics from YAML and inverted to sensor->camera");
      std::stringstream ss;
      ss << R_sc_static_.format(Eigen::IOFormat(4, 0, ", ", "\n", "[", "]"));
      RCLCPP_INFO(this->get_logger(), "R_sensor_camera:\n%s", ss.str().c_str());
      RCLCPP_INFO(this->get_logger(), "t_sensor_camera: [%.6f, %.6f, %.6f]", 
        t_sc_static_.x(), t_sc_static_.y(), t_sc_static_.z());
    }
    rotate_twist_ = this->get_parameter("rotate_twist").as_bool();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_imu_ = this->create_publisher<nav_msgs::msg::Odometry>(imu_out_, 10);
    pub_lio_ = this->create_publisher<nav_msgs::msg::Odometry>(lio_out_, 10);
    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_out_, 2);
    pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(scan_out_, 2);
    pub_scan_color_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(scan_color_out_, 2);
    pub_imu_path_ = this->create_publisher<nav_msgs::msg::Path>("/state_estimation_aligned_path", 10);
    pub_lo_path_ = this->create_publisher<nav_msgs::msg::Path>("/laser_odometry_aligned_path", 10);

    sub_imu_ = this->create_subscription<nav_msgs::msg::Odometry>(
      imu_in_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->odomCb(msg, true); });
    sub_lio_ = this->create_subscription<nav_msgs::msg::Odometry>(
      lio_in_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->odomCb(msg, false); });
    sub_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(map_in_, 2,
      std::bind(&FrameNormalizer::mapCb, this, std::placeholders::_1));
    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(scan_in_, 5,
      std::bind(&FrameNormalizer::scanCb, this, std::placeholders::_1));

    // Image and camera info subscriptions for colorization
    RCLCPP_INFO(this->get_logger(), "=== Creating image subscription ===");
    RCLCPP_INFO(this->get_logger(), "use_compressed_image_ value: %s (bool: %d)", 
      use_compressed_image_ ? "true" : "false", static_cast<int>(use_compressed_image_));
    RCLCPP_INFO(this->get_logger(), "About to check: if (use_compressed_image_) ...");
    
    if (use_compressed_image_) {
      RCLCPP_INFO(this->get_logger(), "ENTERING compressed image branch!");
      // Use best_effort QoS for compressed images (common for high-rate image topics)
      rclcpp::QoS image_qos(5);
      image_qos.best_effort();
      image_qos.durability_volatile();
      sub_image_compressed_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        image_in_, image_qos, std::bind(&FrameNormalizer::compressedImageCb, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "✓ Created compressed image subscription on topic: %s (QoS: best_effort)", image_in_.c_str());
      if (!sub_image_compressed_) {
        RCLCPP_ERROR(this->get_logger(), "✗ FAILED to create compressed image subscription!");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "ENTERING raw image branch (use_compressed_image_ is FALSE)!");
      RCLCPP_WARN(this->get_logger(), "This means the parameter was not loaded correctly from YAML!");
      rclcpp::QoS image_qos(5);
      image_qos.best_effort();
      image_qos.durability_volatile();
      sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_in_, image_qos, std::bind(&FrameNormalizer::imageCb, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "✓ Created raw image subscription on topic: %s (QoS: best_effort)", image_in_.c_str());
      if (!sub_image_) {
        RCLCPP_ERROR(this->get_logger(), "✗ FAILED to create raw image subscription!");
      }
    }
    RCLCPP_INFO(this->get_logger(), "=== Finished creating image subscription ===");
    sub_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(cam_info_in_, 5,
      std::bind(&FrameNormalizer::camInfoCb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "✓ Created camera info subscription on topic: %s", cam_info_in_.c_str());

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&FrameNormalizer::refreshTransform, this));
    
    // Diagnostic timer to check subscription status
    diagnostic_timer_ = this->create_wall_timer(std::chrono::seconds(5),
      std::bind(&FrameNormalizer::diagnosticCheck, this));
  }
  
  void diagnosticCheck() {
    std::lock_guard<std::mutex> lk(img_mutex_);
    if (use_compressed_image_) {
      if (sub_image_compressed_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "Compressed image subscription active on topic '%s'. Buffer size: %zu", 
          image_in_.c_str(), image_buffer_.size());
        if (image_buffer_.empty()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No images received yet on compressed image topic: %s. Check if topic is publishing.", image_in_.c_str());
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Compressed image subscription is NULL!");
      }
    } else {
      if (sub_image_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "Raw image subscription active on topic '%s'. Buffer size: %zu", 
          image_in_.c_str(), image_buffer_.size());
        if (image_buffer_.empty()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No images received yet on raw image topic: %s. Check if topic is publishing.", image_in_.c_str());
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Raw image subscription is NULL!");
      }
    }
  }

private:
  // ============================================================================
  // Callback Functions
  // ============================================================================

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(img_mutex_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
      "Received image at t=%.3f", rclcpp::Time(msg->header.stamp).seconds());

    image_buffer_.push_back(msg);
    constexpr size_t MAX_BUFFER_SIZE = 50;
    if (image_buffer_.size() > MAX_BUFFER_SIZE) {
      image_buffer_.pop_front();
    }
  }

  void compressedImageCb(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  

    try {
      // Decode compressed image to cv::Mat
      cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      if (img.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Failed to decode compressed image");
        return;
      }

      // Convert to sensor_msgs::Image
      auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
      img_msg->header = msg->header;
      img_msg->height = img.rows;
      img_msg->width = img.cols;
      img_msg->encoding = "bgr8";
      img_msg->is_bigendian = false;
      img_msg->step = img.cols * img.elemSize();
      img_msg->data.assign(img.data, img.data + img.total() * img.elemSize());

      image_buffer_.push_back(img_msg);
      constexpr size_t MAX_BUFFER_SIZE = 50;
      if (image_buffer_.size() > MAX_BUFFER_SIZE) {
        image_buffer_.pop_front();
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error decoding compressed image: %s", e.what());
    }
  }

  void camInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(cam_mutex_);
    cam_info_ = msg;
    // Cache intrinsics
    K_ << msg->k[0], msg->k[1], msg->k[2],
          msg->k[3], msg->k[4], msg->k[5],
          msg->k[6], msg->k[7], msg->k[8];
    have_cam_ = true;
  }

  // ============================================================================
  // Helper Functions
  // ============================================================================

  // Project a point in camera frame to pixel (floating-point for interpolation)
  bool projectPointToPixel(const Eigen::Vector3d &p_c, double &u, double &v) const {
    if (p_c.z() <= 0.0) return false;
    double x = p_c.x() / p_c.z();
    double y = p_c.y() / p_c.z();
    double fu = K_(0,0), fv = K_(1,1), cu = K_(0,2), cv = K_(1,2);
    u = fu * x + cu;
    v = fv * y + cv;
    if (!cam_info_) return false;
    return (u >= 0 && v >= 0 && u < static_cast<double>(cam_info_->width - 1) && v < static_cast<double>(cam_info_->height - 1));
  }

  // Get closest image to timestamp within tolerance
  sensor_msgs::msg::Image::ConstSharedPtr getSynchronizedImage(const rclcpp::Time &t) {
    std::lock_guard<std::mutex> lk(img_mutex_);
    if (image_buffer_.empty()) return nullptr;
    
    sensor_msgs::msg::Image::ConstSharedPtr closest_img = nullptr;
    double min_time_diff = std::numeric_limits<double>::max();
    
    for (const auto &img : image_buffer_) {
      double time_diff = std::abs((rclcpp::Time(img->header.stamp) - t).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        closest_img = img;
      }
    }
    
    if (min_time_diff <= sync_tolerance_) {
      RCLCPP_DEBUG(this->get_logger(), "Time synced between image and lidar: %.3fs", min_time_diff);
      return closest_img;
    }

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Time not synced between image and lidar: %.3fs", min_time_diff);
    return nullptr;
  }

  // Get closest odometry to timestamp within tolerance
  nav_msgs::msg::Odometry::ConstSharedPtr getSynchronizedOdom(const rclcpp::Time &t) {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    if (odom_buffer_.empty()) return nullptr;

    nav_msgs::msg::Odometry::ConstSharedPtr closest_odom = nullptr;
    double min_time_diff = std::numeric_limits<double>::max();

    for (const auto &odom : odom_buffer_) {
      double time_diff = std::abs((rclcpp::Time(odom->header.stamp) - t).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        closest_odom = odom;
      }
    }

    if (min_time_diff <= sync_tolerance_) {
      RCLCPP_DEBUG(this->get_logger(), "Time synced between odom and scan: %.3fs", min_time_diff);
      return closest_odom;
    }

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Time not synced between odom and scan: %.3fs", min_time_diff);
    return nullptr;
  }

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
    // Buffer lidar odometry for time synchronization with scans (for colorization)
    if (!is_imu) {
      std::lock_guard<std::mutex> lk(odom_mutex_);
      odom_buffer_.push_back(msg);
      constexpr size_t MAX_BUFFER_SIZE = 50;
      if (odom_buffer_.size() > MAX_BUFFER_SIZE) {
        odom_buffer_.pop_front();
      }
    }

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
      updatePath(out, lo_path_, pub_lo_path_, false);
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

    // Attempt colorization if we have camera info & image
    if (!have_cam_) return;
    auto img = getSynchronizedImage(rclcpp::Time(msg->header.stamp));

    if (!img) return;

    // Get synchronized odometry to get inverse transform (world -> sensor)
    auto odom = getSynchronizedOdom(rclcpp::Time(msg->header.stamp));
    if (!odom) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "No synchronized odometry for colorization");
      return;
    }

    // Extract odometry transform (sensor -> world)
    Eigen::Quaterniond q_odom(odom->pose.pose.orientation.w,
                              odom->pose.pose.orientation.x,
                              odom->pose.pose.orientation.y,
                              odom->pose.pose.orientation.z);
    Eigen::Matrix3d R_world_sensor = q_odom.toRotationMatrix();
    Eigen::Vector3d t_world_sensor(odom->pose.pose.position.x,
                                    odom->pose.pose.position.y,
                                    odom->pose.pose.position.z);

    // Compute inverse transform (world -> sensor)
    Eigen::Matrix3d R_sensor_world = R_world_sensor.transpose();
    Eigen::Vector3d t_sensor_world = -R_sensor_world * t_world_sensor;

    // Lookup transform from sensor frame (point cloud) to camera frame at msg time
    Eigen::Matrix3d R_sc;
    Eigen::Vector3d t_sc = Eigen::Vector3d::Zero();
    if (use_static_extrinsics_) {
      R_sc = R_sc_static_;
      t_sc = t_sc_static_;
    } else {
      try {
        auto ts = tf_buffer_->lookupTransform(camera_frame_, frame_sensor_, msg->header.stamp);
        Eigen::Isometry3d T = tf2::transformToEigen(ts.transform);
        R_sc = T.rotation();
        t_sc = T.translation();
      } catch (const std::exception &) {
        return;
      }
    }

    // Prepare an XYZRGB cloud
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    color_cloud.reserve(cloud_in.size());

    // Image view
    const uint32_t w = img->width, h = img->height;
    const bool rgb8 = (img->encoding == "rgb8");
    const bool bgr8 = (img->encoding == "bgr8");
    const bool mono8 = (img->encoding == "mono8");
    const auto &data = img->data;
    const size_t step = img->step;

    // Helper to get pixel color at integer coordinates
    auto get_pixel = [&](int u, int v) -> std::tuple<uint8_t,uint8_t,uint8_t> {
      if (u < 0 || v < 0 || static_cast<uint32_t>(u) >= w || static_cast<uint32_t>(v) >= h) return {0,0,0};
      const uint8_t *row = &data[static_cast<size_t>(v) * step];
      if (rgb8) {
        return {row[3*u+0], row[3*u+1], row[3*u+2]};
      } else if (bgr8) {
        return {row[3*u+2], row[3*u+1], row[3*u+0]};
      } else if (mono8) {
        uint8_t g = row[u];
        return {g,g,g};
      }
      return {0,0,0};
    };

    // Bilinear interpolation for RGB color
    auto fetch_color_interpolated = [&](double u, double v) -> std::tuple<uint8_t,uint8_t,uint8_t> {
      // Get integer and fractional parts
      const int u_floor = static_cast<int>(std::floor(u));
      const int v_floor = static_cast<int>(std::floor(v));
      const double du = u - u_floor;
      const double dv = v - v_floor;

      // Boundary check
      if (u_floor < 0 || v_floor < 0 || u_floor >= static_cast<int>(w) - 1 || v_floor >= static_cast<int>(h) - 1) {
        return get_pixel(static_cast<int>(u), static_cast<int>(v));
      }

      // Calculate bilinear weights
      const double w_tl = (1.0 - du) * (1.0 - dv);
      const double w_tr = du * (1.0 - dv);
      const double w_bl = (1.0 - du) * dv;
      const double w_br = du * dv;

      // Get four neighboring pixels
      auto [r_tl, g_tl, b_tl] = get_pixel(u_floor, v_floor);
      auto [r_tr, g_tr, b_tr] = get_pixel(u_floor + 1, v_floor);
      auto [r_bl, g_bl, b_bl] = get_pixel(u_floor, v_floor + 1);
      auto [r_br, g_br, b_br] = get_pixel(u_floor + 1, v_floor + 1);

      // Interpolate each color channel
      double r = w_tl * r_tl + w_tr * r_tr + w_bl * r_bl + w_br * r_br;
      double g = w_tl * g_tl + w_tr * g_tr + w_bl * g_bl + w_br * g_br;
      double b = w_tl * b_tl + w_tr * b_tr + w_bl * b_bl + w_br * b_br;

      return {static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)};
    };

    // Colorize point cloud with bilinear interpolation
    // Use cloud_in for camera projection (needs sensor frame), but store gravity-aligned points from cloud_out
    // Only publish points that successfully project to camera image (within FOV)
    for (size_t i = 0; i < cloud_in.points.size() && i < cloud_out.points.size(); ++i) {
      const auto &pt_in = cloud_in.points[i];
      const auto &pt_out = cloud_out.points[i];  // Gravity-aligned point
      
      // Point is in world/odometry frame (from registered scan)
      Eigen::Vector3d p_world(pt_in.x, pt_in.y, pt_in.z);

      // Transform to sensor frame using inverse odometry
      Eigen::Vector3d p_sensor = R_sensor_world * p_world + t_sensor_world;

      // Transform to camera frame
      Eigen::Vector3d p_camera = R_sc * p_sensor + t_sc;

      double u = 0.0, v = 0.0;
      
      // Only add point if it successfully projects to camera image (within FOV)
      if (projectPointToPixel(p_camera, u, v)) {
        // Get color from image using bilinear interpolation
        auto [r, g, b] = fetch_color_interpolated(u, v);
        
        // Skip points with black color (0,0,0) - indicates no valid color found
        if (r == 0 && g == 0 && b == 0) {
          continue;
        }
        
        pcl::PointXYZRGB q;
        // Store point in gravity-aligned frame for output
        q.x = pt_out.x; 
        q.y = pt_out.y; 
        q.z = pt_out.z;
        q.r = r; 
        q.g = g; 
        q.b = b;
        
        // Only add points with valid non-black colors to the cloud
        color_cloud.push_back(q);
      }
      // Skip points outside camera FOV - don't add them to color_cloud
    }

    // Publish colored point cloud (only contains points within camera FOV)
    sensor_msgs::msg::PointCloud2 msg_color;
    pcl::toROSMsg(color_cloud, msg_color);
    msg_color.header = msg->header;
    msg_color.header.frame_id = frame_map_;
    pub_scan_color_->publish(msg_color);
  }

  // ============================================================================
  // Member Variables
  // ============================================================================

  // TF and frame names
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  bool have_tf_ = false;
  std::string frame_map_, frame_sensor_, frame_gravity_;

  // Topic names
  std::string imu_in_, lio_in_, map_in_, scan_in_;
  std::string imu_out_, lio_out_, map_out_, scan_out_;
  std::string image_in_, cam_info_in_, camera_frame_, scan_color_out_;

  // ROS2 subscribers and publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_imu_, sub_lio_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_, sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_compressed_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_, pub_lio_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_, pub_scan_, pub_scan_color_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_imu_path_, pub_lo_path_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  nav_msgs::msg::Path imu_path_, lo_path_;

  // Configuration
  bool rotate_twist_ = false;
  double sync_tolerance_ = 0.05;
  bool use_static_extrinsics_ = false;
  bool use_compressed_image_ = false;

  // Camera data and buffers
  std::mutex img_mutex_, cam_mutex_, odom_mutex_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> image_buffer_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odom_buffer_;
  sensor_msgs::msg::CameraInfo::SharedPtr cam_info_;
  Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
  bool have_cam_ = false;

  // Camera extrinsics (sensor -> camera)
  Eigen::Matrix3d R_sc_static_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_sc_static_ = Eigen::Vector3d::Zero();
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameNormalizer>());
  rclcpp::shutdown();
  return 0;
}


