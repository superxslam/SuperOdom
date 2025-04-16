//
// Created by shibo zhao on 2020-09-27.
//

#ifndef super_odometry_FEATUREEXTRACTION_H
#define super_odometry_FEATUREEXTRACTION_H

// #include "super_odometry/logging.h"


#include <cmath>
#include <string>
#include <vector>
#include <sophus/so3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <super_odometry_msgs/msg/laser_feature.hpp>

#include "super_odometry/container/MapRingBuffer.h"
#include "super_odometry/sensor_data/imu/imu_data.h"
#include "super_odometry/sensor_data/pointcloud/point_os.h"
#include "super_odometry/tic_toc.h"
#include "super_odometry/utils/Twist.h"
#include "super_odometry/config/parameter.h"

#include <mutex>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "super_odometry/utils/superodom_utils.h"


namespace super_odometry {


    using std::atan2;
    using std::cos;
    using std::sin;
    std::vector<std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>> all_cloud_buf(2);
     
    constexpr unsigned int BLOCK_TIME_NS = 55296;   // Time in ns for one block (measurement + recharge)
    constexpr std::size_t NUM_BLOCKS = 12;    // Number of blocks in a Velodyne packet
    constexpr double LIDAR_MESSAGE_TIME = (double)(NUM_BLOCKS * BLOCK_TIME_NS * 151) * 1e-9;
    constexpr double IMU_TIME_LENIENCY = 0.1;
   

    struct bounds_t
    {
        double blindFront;
        double blindBack;
        double blindRight;
        double blindLeft;
    };

    struct feature_extraction_config{
        bounds_t box_size;
        int skipFrame;
        int N_SCANS;
        int provide_point_time;
        bool use_dynamic_mask;
        bool use_imu_roll_pitch;
        bool debug_view_enabled;
        float min_range;
        float max_range;
        int filter_point_size;
        SensorType sensor;
        double imu_acc_x_limit;
        double imu_acc_y_limit;
        double imu_acc_z_limit;
    };

    struct ImuMeasurement {
        double timestamp;
        Eigen::Vector3d accel;
        Eigen::Vector3d gyr;
        Eigen::Quaterniond orientation;
    };

    typedef feature_extraction_config feature_extraction_config;

    class featureExtraction : public rclcpp::Node {
    public:

        /* TODO: return this as a parameter */

        static constexpr double scanPeriod = 0.100859904 - 20.736e-6;
        static constexpr double columnTime = 55.296e-6;
        static constexpr double laserTime = 2.304e-6;

        featureExtraction(const rclcpp::NodeOptions & options);

        void initInterface();
  
        template <typename Meas>
        bool synchronize_measurements(MapRingBuffer<Meas> &measureBuf,
                                        MapRingBuffer<pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr> &lidarBuf);

        void imuRemovePointDistortion(double lidar_start_time, double lidar_end_time, MapRingBuffer<Imu::Ptr> &imuBuf,
                                    pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg);

        void vioRemovePointDistortion(double lidar_start_time, double lidar_end_time, MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr>&vioBuf,
                                    pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg);

        void undistortionAndFeatureExtraction();

        void extractFeatures(double lidar_start_time, const pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr& lidar_msg, const Eigen::Quaterniond& quaternion);

        void imu_Handler(const sensor_msgs::msg::Imu::SharedPtr msg_in);

        void visual_odom_Handler(const nav_msgs::msg::Odometry::SharedPtr visualOdometry);

        void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);

        void livoxHandler(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg);

        void uniformFeatureExtraction(const pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &pc_in, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf, int skip_num, float block_range);

        void assignTimeforPointCloud(pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_);
        
        template <typename Point>
        sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, typename pcl::PointCloud<Point>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame);

        bool readParameters();

        void publishTopic(double lidar_start_time, 
                                         pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr laser_no_distortion_points,
                                         pcl::PointCloud<PointType>::Ptr edgePoints,
                                         pcl::PointCloud<PointType>::Ptr plannerPoints, 
                                         pcl::PointCloud<PointType>::Ptr depthPoints,
                                         Eigen::Quaterniond q_w_original_l);

        void manageLidarBuffer(pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud, double timestamp);

        ImuMeasurement parseImuMessage(const sensor_msgs::msg::Imu::SharedPtr& msg);

        double calculateDeltaTime(double current_timestamp);

        Imu::Ptr createImuData(const ImuMeasurement& measurement);

        void updateImuOrientation(Imu::Ptr& imudata);

        void imuInitialization(double timestamp);

        template<typename BufferType>
        void removePointDistortion(
            double lidar_start_time, 
            double lidar_end_time,
            MapRingBuffer<BufferType> &buffer,
            pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg);

       
        
        template<typename BufferType>
        Transformd getInterpolatedPose(double timestamp, MapRingBuffer<BufferType> &buffer,
                                const std::function<Transformd(const BufferType&)>& extractPose);

        Eigen::Vector3d transformPoint(const point_os::PointcloudXYZITR& point, const Transformd& T_w_original,
                                        const Transformd& point_pose, bool is_imu_data);
        
        void updatePointPosition(point_os::PointcloudXYZITR& point, const Eigen::Vector3d& new_pos);

        bool isPointValid(const point_os::PointcloudXYZITR& point);

        
        Imu::Ptr imu_Init = std::make_shared<Imu>();
        MapRingBuffer<Imu::Ptr> imuBuf;
        MapRingBuffer<pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr> lidarBuf;
        MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> visualOdomBuf;


    private:
        // ROS Interface
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
        rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subLivoxCloud;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubEdgePoints;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPlannerPoints;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubBobPoints;
        rclcpp::Publisher<super_odometry_msgs::msg::LaserFeature>::SharedPtr pubLaserFeatureInfo;
        std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubEachScan;

        rclcpp::CallbackGroup::SharedPtr cb_group_;

        int delay_count_;
        std::mutex m_buf;
        int frameCount = 0;

        bool PUB_EACH_LINE = false;
        bool LASER_IMU_SYNC_SCCUESS = false;
        bool LASER_CAMERA_SYNC_SUCCESS = false;
        bool IMU_INIT=false;
        double m_imuPeriod;

        super_odometry_msgs::msg::LaserFeature laserFeature;
        std_msgs::msg::Header FeatureHeader;
        Eigen::Quaterniond q_w_original_l;
        Eigen::Vector3d t_w_original_l;
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloudwithTime=nullptr;
        pcl::PointCloud<point_os::OusterPointXYZIRT>::Ptr tmpOusterCloudIn=nullptr ;
        feature_extraction_config config_;
    };

} // namespace super_odometry

#endif //super_odometry_FEATUREEXTRACTION_H
