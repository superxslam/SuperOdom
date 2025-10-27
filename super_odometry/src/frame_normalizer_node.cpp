#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
    // Colorization params
    this->declare_parameter<std::string>("image_in", "/zed/zed_node/right/image_rect_color/compressed");
    this->declare_parameter<std::string>("camera_info_in", "/zed/zed_node/right/camera_info");
    this->declare_parameter<std::string>("camera_frame", "zed_right_camera_frame");
    this->declare_parameter<std::string>("scan_color_out", "/SuperOdom/registered_scan_color");
    this->declare_parameter<double>("sync_tolerance", 0.05);
    // Static extrinsics option (sensor -> camera)
    this->declare_parameter<bool>("use_static_extrinsics", true);
    this->declare_parameter<double>("cam_tx", 0.10898250807257563);
    this->declare_parameter<double>("cam_ty", 0.11401182341214929);
    this->declare_parameter<double>("cam_tz",  -0.1842058976916049);
    this->declare_parameter<double>("cam_roll_deg", 0.0);
    this->declare_parameter<double>("cam_pitch_deg", 0.0);
    this->declare_parameter<double>("cam_yaw_deg", 0.0);
    this->declare_parameter<double>("cam_qx", -0.48647692564127215);
    this->declare_parameter<double>("cam_qy", 0.010071747888767442);
    this->declare_parameter<double>("cam_qz", 0.006933289155231205);
    this->declare_parameter<double>("cam_qw", 0.8736078583750099);
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
    image_in_ = this->get_parameter("image_in").as_string();
    cam_info_in_ = this->get_parameter("camera_info_in").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    scan_color_out_ = this->get_parameter("scan_color_out").as_string();
    sync_tolerance_ = this->get_parameter("sync_tolerance").as_double();
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
      if (qnorm > 0.5) {
        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();
        R_sc_static_ = q.toRotationMatrix();
      } else {
        const double r = this->get_parameter("cam_roll_deg").as_double() * M_PI / 180.0;
        const double p = this->get_parameter("cam_pitch_deg").as_double() * M_PI / 180.0;
        const double y = this->get_parameter("cam_yaw_deg").as_double() * M_PI / 180.0;
        Eigen::AngleAxisd Rx(r, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(p, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(y, Eigen::Vector3d::UnitZ());
        R_sc_static_ = (Rz * Ry * Rx).toRotationMatrix();
      }
      t_sc_static_ = Eigen::Vector3d(tx, ty, tz);
    }
    rotate_twist_ = this->get_parameter("rotate_twist").as_bool();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_imu_ = this->create_publisher<nav_msgs::msg::Odometry>(imu_out_, 10);
    pub_lio_ = this->create_publisher<nav_msgs::msg::Odometry>(lio_out_, 10);
    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_out_, 2);
    pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(scan_out_, 2);
    pub_scan_color_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(scan_color_out_, 2);

    sub_imu_ = this->create_subscription<nav_msgs::msg::Odometry>(
      imu_in_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->odomCb(msg, true); });
    sub_lio_ = this->create_subscription<nav_msgs::msg::Odometry>(
      lio_in_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->odomCb(msg, false); });
    sub_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(map_in_, 2,
      std::bind(&FrameNormalizer::mapCb, this, std::placeholders::_1));
    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(scan_in_, 5,
      std::bind(&FrameNormalizer::scanCb, this, std::placeholders::_1));

    // Image and camera info for colorization (direct compressed subscription)
    sub_compressed_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      image_in_, 5, std::bind(&FrameNormalizer::compressedImageCb, this, std::placeholders::_1));
    sub_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(cam_info_in_, 5,
      std::bind(&FrameNormalizer::camInfoCb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&FrameNormalizer::refreshTransform, this));
  }

private:
  // Camera data buffers
  void compressedImageCb(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(img_mutex_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, 
      "\033[32m Received compressed image at t=%.3f \033[0m", rclcpp::Time(msg->header.stamp).seconds());
    
    // Decode compressed image to cv::Mat
    try {
      cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      if (img.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to decode compressed image");
        return;
      }
      
      // Convert to sensor_msgs::Image
      auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
      img_msg->header = msg->header;
      img_msg->height = img.rows;
      img_msg->width = img.cols;
      img_msg->encoding = msg->format.find("jpeg") != std::string::npos || msg->format.find("jpg") != std::string::npos ? "bgr8" : "bgr8";
      img_msg->is_bigendian = false;
      img_msg->step = img.cols * img.elemSize();
      img_msg->data.assign(img.data, img.data + img.total() * img.elemSize());
      
      last_image_ = img_msg;
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

  // Project a point in sensor frame to pixel
  bool projectPointToPixel(const Eigen::Vector3d &p_c, int &u, int &v) const {
    if (p_c.z() <= 0.0) return false;
    double x = p_c.x() / p_c.z();
    double y = p_c.y() / p_c.z();
    double fu = K_(0,0), fv = K_(1,1), cu = K_(0,2), cv = K_(1,2);
    u = static_cast<int>(fu * x + cu);
    v = static_cast<int>(fv * y + cv);
    if (!cam_info_) return false;
    return (u >= 0 && v >= 0 && u < static_cast<int>(cam_info_->width) && v < static_cast<int>(cam_info_->height));
  }

  // Get closest image to timestamp within tolerance
  sensor_msgs::msg::Image::ConstSharedPtr getSynchronizedImage(const rclcpp::Time &t) {
    std::lock_guard<std::mutex> lk(img_mutex_);
    if (!last_image_) return nullptr;
    rclcpp::Time ti(last_image_->header.stamp);
    double time_diff=(ti - t).seconds();
    if (std::abs(time_diff) <= sync_tolerance_)
     {
       RCLCPP_INFO_STREAM(this->get_logger(), "\033[32m Time synced between image and lidar: " << time_diff << " \033[0m");
       return last_image_;
     }
    else
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "\033[31m Time not synced between image and lidar: " << time_diff << " \033[0m");
    }
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
    if (is_imu) pub_imu_->publish(out);
    else        pub_lio_->publish(out);
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

    // Lookup transform from sensor frame (point cloud) to camera frame at msg time
    Eigen::Matrix3d R_sc;
    Eigen::Vector3d t_sc = Eigen::Vector3d::Zero();
    if (use_static_extrinsics_) {
      R_sc = R_sc_static_;
      std::cout << "R_sc_static_: " << R_sc_static_ << std::endl;
      std::cout << "t_sc_static_: " << t_sc_static_ << std::endl;
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

    auto fetch_color = [&](int u, int v) -> std::tuple<uint8_t,uint8_t,uint8_t> {
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

    for (const auto &pt : cloud_in.points) {
      Eigen::Vector3d ps(pt.x, pt.y, pt.z);
      Eigen::Vector3d pc = R_sc * ps + t_sc; // sensor -> camera
      int u=0, v=0;
      pcl::PointXYZRGB q;
      q.x = ps.x(); q.y = ps.y(); q.z = ps.z();
      if (projectPointToPixel(pc, u, v)) {
        auto [r,g,b] = fetch_color(u,v);
        q.r = r; q.g = g; q.b = b;
      } else {
        q.r = q.g = q.b = 0;
      }
      color_cloud.push_back(q);
    }

    sensor_msgs::msg::PointCloud2 msg_color;
    pcl::toROSMsg(color_cloud, msg_color);
    msg_color.header = msg->header;
    msg_color.header.frame_id = frame_map_;
    pub_scan_color_->publish(msg_color);
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
  std::string image_in_, cam_info_in_, camera_frame_, scan_color_out_;
  double sync_tolerance_ {0.05};

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_imu_, sub_lio_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_, pub_lio_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_color_;
  bool rotate_twist_ = false;
  rclcpp::TimerBase::SharedPtr timer_;

  // Camera cache
  std::mutex img_mutex_, cam_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr last_image_;
  sensor_msgs::msg::CameraInfo::SharedPtr cam_info_;
  Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
  bool have_cam_ = false;

  // Image/colorization config
  bool use_static_extrinsics_ {false};
  Eigen::Matrix3d R_sc_static_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_sc_static_ = Eigen::Vector3d::Zero();
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameNormalizer>());
  rclcpp::shutdown();
  return 0;
}


