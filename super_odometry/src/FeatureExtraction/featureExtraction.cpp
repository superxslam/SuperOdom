//
// Created by shibo zhao on 2020-09-27.
//

#include <super_odometry/FeatureExtraction/featureExtraction.h>
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

#define BOLD "\033[1m"
#define UNDERLINE "\033[4m"
#define ITALIC "\033[3m"


namespace super_odometry {
    
    featureExtraction::featureExtraction(const rclcpp::NodeOptions & options)
    : Node("feature_extraction_node", options) {
    }

    void featureExtraction::initInterface() {      
        //! Callback Groups
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

        rclcpp::QoS imu_qos(10);
        imu_qos.best_effort();  // Use BEST_EFFORT reliability
        imu_qos.keep_last(10);  // Keep last 10 messages

        rclcpp::QoS laser_qos(10);
        laser_qos.best_effort();  // Use BEST_EFFORT reliability
        laser_qos.keep_last(2);  // Keep last 10 messages

        if(!readGlobalparam(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[super_odometry::featureExtraction] Could not read calibration. Exiting...");
            rclcpp::shutdown();
        }
        if (!readParameters())
        {
            RCLCPP_ERROR(this->get_logger(), "[super_odometry::featureExtraction] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "calibration");
        if (!readCalibration(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[super_odometry::featureExtraction] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }
         
        RCLCPP_WARN(this->get_logger(), "config_.skipFrame: %d", config_.skipFrame);
        RCLCPP_INFO(this->get_logger(), "scan line number %d \n", config_.N_SCANS);      
        RCLCPP_INFO(this->get_logger(), "use imu roll and pitch %d \n", config_.use_imu_roll_pitch);

        if (config_.N_SCANS != 16 && config_.N_SCANS != 32 && config_.N_SCANS != 64 && config_.N_SCANS != 4 && config_.N_SCANS != 128)
        {
            RCLCPP_ERROR(this->get_logger(), "only support velodyne, livox, ouster with 16, 32, 64 or 128 scan line! and livox mid 360");
            rclcpp::shutdown();
        }

        if (config_.sensor == SensorType::VELODYNE || config_.sensor == SensorType::OUSTER) {
            subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(LASER_TOPIC, laser_qos, 
                    std::bind(&featureExtraction::laserCloudHandler, this,
                    std::placeholders::_1), sub_options);
        } else if (config_.sensor == SensorType::LIVOX) {
            subLivoxCloud = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(LASER_TOPIC, 20, 
                    std::bind(&featureExtraction::livoxHandler, this,
                    std::placeholders::_1), sub_options);
        } //TODO: add this to config

        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, imu_qos, 
            std::bind(&featureExtraction::imu_Handler, this,
                        std::placeholders::_1), sub_options);

        subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
            ODOM_TOPIC, 10, 
            std::bind(&featureExtraction::visual_odom_Handler, this,
                        std::placeholders::_1), sub_options);

        pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/velodyne_cloud_2", 2);

        pubLaserFeatureInfo = this->create_publisher<super_odometry_msgs::msg::LaserFeature>(
            ProjectName+"/feature_info", 2);

        pubBobPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/bob_points", 2);

        pubPlannerPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/planner_points", 2);

        pubEdgePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/edge_points", 2);

        if (PUB_EACH_LINE)
        {
            for (int i = 0; i < config_.N_SCANS; i++)
            {
                auto tmp_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "laser_scanid_" + std::to_string(i), 2);
                pubEachScan.push_back(tmp_publisher_);
            }
        }

        delay_count_ = 0;
        m_imuPeriod = 1.0/imu_Init->imu_frequency;
    }

    bool featureExtraction::readParameters()
    {          

        this->declare_parameter<int>("feature_extraction_node.scan_line", 4);
        this->declare_parameter<int>("feature_extraction_node.mapping_skip_frame", 1);
        this->declare_parameter<double>("feature_extraction_node.blindFront", 0.1);
        this->declare_parameter<double>("feature_extraction_node.blindBack", -1.0);
        this->declare_parameter<double>("feature_extraction_node.blindLeft", 0.1);
        this->declare_parameter<double>("feature_extraction_node.blindRight", -0.1);
        this->declare_parameter<bool>("feature_extraction_node.use_dynamic_mask", false);
        this->declare_parameter<bool>("feature_extraction_node.use_imu_roll_pitch", false);
        this->declare_parameter<float>("feature_extraction_node.min_range", 0.2);
        this->declare_parameter<float>("feature_extraction_node.max_range", 130.0);
        this->declare_parameter<int>("feature_extraction_node.filter_point_size", 3);
        this->declare_parameter<int>("feature_extraction_node.provide_point_time", 1);
        this->declare_parameter<bool>("feature_extraction_node.debug_view", false);
        this->declare_parameter<double>("feature_extraction_node.imu_acc_x_limit", 1.0);
        this->declare_parameter<double>("feature_extraction_node.imu_acc_y_limit", 1.0);
        this->declare_parameter<double>("feature_extraction_node.imu_acc_z_limit", 1.0);
        this->declare_parameter<std::string>("feature_extraction_node.sensor", "livox");

                
        config_.N_SCANS = this->get_parameter("feature_extraction_node.scan_line").as_int();
        config_.skipFrame = this->get_parameter("feature_extraction_node.mapping_skip_frame").as_int();
        config_.box_size.blindFront = this->get_parameter("feature_extraction_node.blindFront").as_double();
        config_.box_size.blindBack = this->get_parameter("feature_extraction_node.blindBack").as_double();
        config_.box_size.blindLeft = this->get_parameter("feature_extraction_node.blindLeft").as_double();
        config_.box_size.blindRight = this->get_parameter("feature_extraction_node.blindRight").as_double();
        // config_.use_imu_roll_pitch = this->get_parameter("feature_extraction_node.use_imu_roll_pitch").as_bool();
        config_.min_range = this->get_parameter("feature_extraction_node.min_range").as_double();
        config_.max_range = this->get_parameter("feature_extraction_node.max_range").as_double();
        config_.filter_point_size = this->get_parameter("feature_extraction_node.filter_point_size").as_int();
        config_.provide_point_time = this->get_parameter("feature_extraction_node.provide_point_time").as_int();
        config_.use_dynamic_mask = this->get_parameter("feature_extraction_node.use_dynamic_mask").as_bool(); 
        config_.debug_view_enabled = this->get_parameter("feature_extraction_node.debug_view").as_bool();
        config_.imu_acc_x_limit = this->get_parameter("feature_extraction_node.imu_acc_x_limit").as_double();
        config_.imu_acc_y_limit = this->get_parameter("feature_extraction_node.imu_acc_y_limit").as_double();
        config_.imu_acc_z_limit = this->get_parameter("feature_extraction_node.imu_acc_z_limit").as_double();
        config_.use_imu_roll_pitch = USE_IMU_ROLL_PITCH;
        config_.imu_acc_x_limit = IMU_ACC_X_LIMIT;
        config_.imu_acc_y_limit = IMU_ACC_Y_LIMIT;
        config_.imu_acc_z_limit = IMU_ACC_Z_LIMIT;

        if (SENSOR == "livox") {
            config_.sensor = SensorType::LIVOX;
        } else if (SENSOR == "velodyne") {
            config_.sensor = SensorType::VELODYNE;
        } else if (SENSOR == "ouster") {
            config_.sensor = SensorType::OUSTER;
        } 
        return true;
    }


    template <typename Meas>
    bool featureExtraction::synchronize_measurements(MapRingBuffer<Meas> &measureBuf,
                                                     MapRingBuffer<pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr> &lidarBuf)
    {

        if (lidarBuf.getSize() == 0 or measureBuf.getSize() == 0)
            return false;

        double lidar_start_time;
        lidarBuf.getFirstTime(lidar_start_time);

        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;
        lidarBuf.getFirstMeas(lidar_msg);

        double lidar_end_time = lidar_start_time + lidar_msg->back().time;

        // obtain the current imu message
        double meas_start_time=0;
        measureBuf.getFirstTime(meas_start_time);

        double meas_end_time=0;
        measureBuf.getLastTime(meas_end_time);

        if (meas_end_time <= lidar_end_time) // make sure imu message arrives after lidar message
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "meas_end_time < lidar_end_time ||"
                            " message order is not perfect! please restart velodyne and imu driver!");
            RCLCPP_WARN(this->get_logger(), "meas_end_time %f <  %f lidar_end_time", meas_end_time, lidar_end_time);
            RCLCPP_WARN(this->get_logger(), "All the lidar data is more recent than all the imu data. Will throw away lidar frame");

            return false;
        }

        if (meas_start_time >= lidar_start_time)          
        {
            RCLCPP_WARN(this->get_logger(), "throw laser scan, only should happen at the beginning");
            lidarBuf.clean(lidar_start_time);
            RCLCPP_WARN(this->get_logger(), "removed the lidarBuf size % d, measureBuf size % d ", lidarBuf.getSize(), measureBuf.getSize());
            RCLCPP_WARN(this->get_logger(), "meas_start_time: %f > lidar_start_time: %f ", meas_start_time, lidar_start_time);
            return false;
        }
        else
        {
            return true;
        }
        
    }


    

    template<typename BufferType>
    void featureExtraction::removePointDistortion(
        double lidar_start_time, 
        double lidar_end_time,
        MapRingBuffer<BufferType> &buffer,
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg)
    {
        // Step 1: Define how to extract pose based on buffer type
        auto extractPose = [](const BufferType& data) -> Transformd {
            Transformd pose;
            if constexpr (std::is_same_v<BufferType, Imu::Ptr>) {
                // For IMU data: only rotation, zero translation
                pose.rot = data->q_w_i;
                pose.pos = Eigen::Vector3d::Zero();
            } else {
                // For VIO data: both rotation and translation
                pose.rot = Eigen::Quaterniond(
                    data->pose.pose.orientation.w,
                    data->pose.pose.orientation.x,
                    data->pose.pose.orientation.y,
                    data->pose.pose.orientation.z
                );
                pose.pos = Eigen::Vector3d(
                    data->pose.pose.position.x,
                    data->pose.pose.position.y,
                    data->pose.pose.position.z
                );
            }
            return pose;
        };

        
         // Step 2: Get interpolated poses directly
        auto getInterpolatedPoseAtTime = [&buffer, &extractPose](double timestamp) -> Transformd {
        auto after_ptr = buffer.measMap_.upper_bound(timestamp);
        if (after_ptr->first < 0.0001) {
            after_ptr = buffer.measMap_.begin();
        }

        if (after_ptr == buffer.measMap_.begin()) {
            return extractPose(after_ptr->second);
        }

        auto before_ptr = std::prev(after_ptr);
        double ratio = (timestamp - before_ptr->first) / 
                      (after_ptr->first - before_ptr->first);

        Transformd before_pose = extractPose(before_ptr->second);
        Transformd after_pose = extractPose(after_ptr->second);

        Transformd result;
        result.rot = before_pose.rot.slerp(ratio, after_pose.rot);
        result.pos = (1 - ratio) * before_pose.pos + ratio * after_pose.pos;
        return result;
    };

    // Step 3: Get start pose
    Transformd start_pose = getInterpolatedPoseAtTime(lidar_start_time);

    // Step 4: Calculate initial transform
    Transformd T_w_original(start_pose.rot, start_pose.pos);
    bool is_imu_data = std::is_same_v<BufferType, Imu::Ptr>;
    Transformd T_w_original_sensor = is_imu_data ? 
                                    T_w_original * T_i_l : 
                                    T_w_original;

    q_w_original_l = T_w_original_sensor.rot;
    t_w_original_l = T_w_original_sensor.pos;

    // Step 5: Process each point
    for (auto &point : lidar_msg->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        double point_time = point.time + lidar_start_time;
        Transformd point_pose = getInterpolatedPoseAtTime(point_time);
        
        // Transform point
        Transformd T_w_current(point_pose.rot, point_pose.pos);
        Transformd T_original_current = T_w_original.inverse() * T_w_current;
        Transformd T_final = is_imu_data ? 
                            T_l_i * T_original_current * T_i_l : 
                            T_original_current;

        Eigen::Vector3d pt(point.x, point.y, point.z);
        pt = T_final * pt;
            
        point.x = pt.x();
        point.y = pt.y();
        point.z = pt.z();
        }
    }

    

    // Helper functions for clarity and reusability
    template<typename BufferType>
    Transformd featureExtraction::getInterpolatedPose(
        double timestamp, 
        MapRingBuffer<BufferType> &buffer,
        const std::function<Transformd(const BufferType&)>& extractPose)
    {
        auto after_ptr = buffer.measMap_.upper_bound(timestamp);
        if (after_ptr->first < 0.0001) {
            after_ptr = buffer.measMap_.begin();
        }

        if (after_ptr == buffer.measMap_.begin()) {
            return extractPose(after_ptr->second);
        }

        auto before_ptr = std::prev(after_ptr);
        double ratio = (timestamp - before_ptr->first) / 
                    (after_ptr->first - before_ptr->first);

        Transformd before_pose = extractPose(before_ptr->second);
        Transformd after_pose = extractPose(after_ptr->second);

        Transformd result;
        result.rot = before_pose.rot.slerp(ratio, after_pose.rot);
        result.pos = (1 - ratio) * before_pose.pos + ratio * after_pose.pos;
        return result;
    }

    bool featureExtraction::isPointValid(const point_os::PointcloudXYZITR& point) {
        return std::isfinite(point.x) && 
            std::isfinite(point.y) && 
            std::isfinite(point.z);
    }

    Eigen::Vector3d featureExtraction::transformPoint(
        const point_os::PointcloudXYZITR& point,
        const Transformd& T_w_original,
        const Transformd& point_pose,
        bool is_imu_data)
    {
        Transformd T_w_current(point_pose.rot, point_pose.pos);
        Transformd T_original_current = T_w_original.inverse() * T_w_current;

        Transformd T_final = is_imu_data ? 
                            T_l_i * T_original_current * T_i_l : 
                            T_original_current;

        Eigen::Vector3d pt(point.x, point.y, point.z);
        return T_final * pt;
    }

    void featureExtraction::updatePointPosition(point_os::PointcloudXYZITR& point, const Eigen::Vector3d& new_pos) {
        point.x = new_pos.x();
        point.y = new_pos.y();
        point.z = new_pos.z();
    }


    template <typename Point>
    sensor_msgs::msg::PointCloud2
    featureExtraction::publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, typename pcl::PointCloud<Point>::Ptr thisCloud,
                                    rclcpp::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::msg::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        return tempCloud;
    }

    void featureExtraction::publishTopic(double lidar_start_time, 
                                         pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr laser_no_distortion_points,
                                         pcl::PointCloud<PointType>::Ptr edgePoints,
                                         pcl::PointCloud<PointType>::Ptr plannerPoints, 
                                         pcl::PointCloud<PointType>::Ptr depthPoints,
                                         Eigen::Quaterniond q_w_original_l)
    {
        FeatureHeader.frame_id = WORLD_FRAME;
        FeatureHeader.stamp = rclcpp::Time(lidar_start_time*1e9);
        laserFeature.header = FeatureHeader;
        laserFeature.imu_available = false;
        laserFeature.odom_available = false;

      
        laserFeature.cloud_nodistortion = publishCloud<point_os::PointcloudXYZITR>(pubLaserCloud, laser_no_distortion_points, FeatureHeader.stamp, SENSOR_FRAME);
        laserFeature.cloud_corner = publishCloud<PointType>(pubEdgePoints, edgePoints, FeatureHeader.stamp, SENSOR_FRAME);
        laserFeature.cloud_surface = publishCloud<PointType>(pubPlannerPoints, plannerPoints, FeatureHeader.stamp, SENSOR_FRAME);
        laserFeature.cloud_realsense=publishCloud<PointType>(pubBobPoints, depthPoints, FeatureHeader.stamp, SENSOR_FRAME);
       
        laserFeature.initial_quaternion_x = q_w_original_l.x();
        laserFeature.initial_quaternion_y = q_w_original_l.y();
        laserFeature.initial_quaternion_z = q_w_original_l.z();
        laserFeature.initial_quaternion_w= q_w_original_l.w();

        laserFeature.initial_pose_x = t_w_original_l.x();
        laserFeature.initial_pose_y = t_w_original_l.y();
        laserFeature.initial_pose_z = t_w_original_l.z();

        laserFeature.imu_available = true;
        laserFeature.sensor = 0; 
        pubLaserFeatureInfo->publish(laserFeature);
    }

    void featureExtraction::extractFeatures(
        double lidar_start_time,
        const pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr& lidar_msg,
        const Eigen::Quaterniond& quaternion)
    {
        pcl::PointCloud<PointType>::Ptr plannerPoints(new pcl::PointCloud<PointType>());
        plannerPoints->reserve(lidar_msg->points.size());
        pcl::PointCloud<PointType>::Ptr edgePoints(new pcl::PointCloud<PointType>());
        edgePoints->reserve(lidar_msg->points.size());
        pcl::PointCloud<PointType>::Ptr bobPoints(new pcl::PointCloud<PointType>());
        bobPoints->reserve(lidar_msg->points.size());

        uniformFeatureExtraction(lidar_msg, plannerPoints, config_.filter_point_size, config_.min_range);
        
        publishTopic(lidar_start_time, lidar_msg, edgePoints, plannerPoints, bobPoints, quaternion);
    }


    void featureExtraction::undistortionAndFeatureExtraction()      
    {
        LASER_IMU_SYNC_SCCUESS = synchronize_measurements<Imu::Ptr>(imuBuf, lidarBuf);
        LASER_CAMERA_SYNC_SUCCESS = synchronize_measurements<nav_msgs::msg::Odometry::SharedPtr>(visualOdomBuf, lidarBuf);

        if (frameCount > 100 and LASER_CAMERA_SYNC_SUCCESS == true)
            LASER_CAMERA_SYNC_SUCCESS = true;
        else
            LASER_CAMERA_SYNC_SUCCESS = false;

        if ((LASER_IMU_SYNC_SCCUESS == true or LASER_CAMERA_SYNC_SUCCESS == true) and lidarBuf.getSize() > 0)
        {
            double lidar_start_time;
            lidarBuf.getFirstTime(lidar_start_time);
            pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;
            lidarBuf.getFirstMeas(lidar_msg);

            double lidar_end_time = lidar_start_time + lidar_msg->back().time;

            if (LASER_IMU_SYNC_SCCUESS == true and LASER_CAMERA_SYNC_SUCCESS == true)
            {
                RCLCPP_INFO(this->get_logger(), "\033[1;32m----> Both IMU ,VIO laserscan are synchronized!.\033[0m");
                removePointDistortion<nav_msgs::msg::Odometry::SharedPtr>(lidar_start_time, lidar_end_time, visualOdomBuf, lidar_msg);
            }

            if (LASER_IMU_SYNC_SCCUESS == false and LASER_CAMERA_SYNC_SUCCESS == true)
            {
                removePointDistortion<nav_msgs::msg::Odometry::SharedPtr>(lidar_start_time, lidar_end_time, visualOdomBuf, lidar_msg);
            }

            if (LASER_IMU_SYNC_SCCUESS == true and LASER_CAMERA_SYNC_SUCCESS == false)
            {
                // RCLCPP_INFO(this->get_logger(), "\033[1;32m----> IMU and laserscan is synchronized!.\033[0m");
                removePointDistortion<Imu::Ptr>(lidar_start_time, lidar_end_time, imuBuf, lidar_msg);
            }

            // Extract features and publish
            extractFeatures(lidar_start_time, lidar_msg, q_w_original_l);

            LASER_CAMERA_SYNC_SUCCESS = false;
            LASER_IMU_SYNC_SCCUESS = false;
        
        }
        else if (imuBuf.empty())
        {
            double lidar_start_time;
            lidarBuf.getFirstTime(lidar_start_time);
            pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;
            lidarBuf.getFirstMeas(lidar_msg);
            double lidar_end_time = lidar_start_time + lidar_msg->back().time;

            RCLCPP_INFO(this->get_logger(), "\033[1;32m----> no IMU data, running LiDAR Odometry only.\033[0m");
            Eigen::Quaterniond default_quaternion = Eigen::Quaterniond::Identity();
            
            // Extract features and publish with default quaternion
            extractFeatures(lidar_start_time, lidar_msg, default_quaternion);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "sync unsuccessfull, skipping scan frame");
        }
        
    }

    void featureExtraction::uniformFeatureExtraction(const pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &pc_in, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf, int skip_num, float block_range)
    {   
        for (uint i=1; i <(int)pc_in->points.size(); i+=skip_num)
        {   
            pcl::PointXYZI point;
            point.x=pc_in->points[i].x;
            point.y=pc_in->points[i].y;
            point.z=pc_in->points[i].z;
            point.intensity=pc_in->points[i].time;

            if ((abs(pc_in->points[i].x - pc_in->points[i-1].x) > 1e-7)
                || (abs(pc_in->points[i].y - pc_in->points[i-1].y) > 1e-7)
                || (abs(pc_in->points[i].z - pc_in->points[i-1].z) > 1e-7)
                && (pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y + pc_in->points[i].z * pc_in->points[i].z > (block_range * block_range)))
            {
                pc_out_surf->push_back(point);
            }
        
        }
        
    }

    ImuMeasurement featureExtraction::parseImuMessage(const sensor_msgs::msg::Imu::SharedPtr& msg) {
        ImuMeasurement measurement;
        measurement.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        measurement.accel << msg->linear_acceleration.x, 
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z;
        measurement.gyr << msg->angular_velocity.x, 
                        msg->angular_velocity.y,
                        msg->angular_velocity.z;
        measurement.orientation = Eigen::Quaterniond(msg->orientation.w,
                                                msg->orientation.x,
                                                msg->orientation.y,
                                                msg->orientation.z);
        return measurement;
    }

    double featureExtraction::calculateDeltaTime(double current_timestamp) {
        double lastImuTime = 0.0;
        double dt = m_imuPeriod;
        
        if(imuBuf.getLastTime(lastImuTime)) {
            dt = current_timestamp - lastImuTime;
            if(abs(dt - m_imuPeriod) > m_imuPeriod * IMU_TIME_LENIENCY) {
                dt = m_imuPeriod; // IMU timestamp jumped - quietly assume normal delta t
            }
        }
        return dt;
    }

    Imu::Ptr featureExtraction::createImuData(const ImuMeasurement& measurement) {
        Imu::Ptr imudata = std::make_shared<Imu>();
        imudata->time = measurement.timestamp;
        
        // Handle Livox sensor specific processing
        if(IMU_INIT && config_.sensor == SensorType::LIVOX) {
            double gravity = imu_Init->gravity_norm;
            Eigen::Vector3d gyr = imu_Init->imu_laser_R_Gravity * measurement.gyr;
            Eigen::Vector3d accel = imu_Init->imu_laser_R_Gravity * measurement.accel;
            imudata->acc = accel * gravity / imu_Init->acc_mean.norm();
        } else {
            imudata->acc = measurement.accel;
        }
        
        imudata->gyr = measurement.gyr;
        return imudata;
    }

    void featureExtraction::updateImuOrientation(Imu::Ptr& imudata) {
        if (!imuBuf.empty()) {
            const auto& last_imu = imuBuf.measMap_.rbegin()->second;
            const double dt = imudata->time - last_imu->time;
            
            Eigen::Vector3d delta_angle = dt * 0.5 * (imudata->gyr + last_imu->gyr);
            Eigen::Quaterniond delta_r = Sophus::SO3d::exp(delta_angle).unit_quaternion();
            
            imudata->q_w_i = last_imu->q_w_i * delta_r;
            imudata->q_w_i.normalize();
        } else if (config_.use_imu_roll_pitch) {
            tf2::Quaternion orientation_curr(imudata->q_w_i.x(),
                                        imudata->q_w_i.y(),
                                        imudata->q_w_i.z(),
                                        imudata->q_w_i.w());
            double roll, pitch, yaw;
            tf2::Matrix3x3(orientation_curr).getRPY(roll, pitch, yaw);
            
            tf2::Quaternion yaw_quat;
            yaw_quat.setRPY(0, 0, -yaw);
            tf2::Quaternion first_orientation = yaw_quat * orientation_curr;
            
            imudata->q_w_i = Eigen::Quaterniond(first_orientation.w(),
                                            first_orientation.x(),
                                            first_orientation.y(),
                                            first_orientation.z());
        }
    }

    void featureExtraction::imuInitialization(double timestamp) {
        double lidar_first_time = 0;
        if(lidarBuf.getFirstTime(lidar_first_time) && 
            timestamp > lidar_first_time + LIDAR_MESSAGE_TIME + 0.05) {
            
            double first_time = 0.0;
            imuBuf.getFirstTime(first_time);
            
            if (timestamp - first_time > 1.0 && !IMU_INIT) {
                imu_Init->imuInit(imuBuf);
                IMU_INIT = true;
                imuBuf.clean(timestamp);
                RCLCPP_INFO(this->get_logger(), "IMU Initialization Process Finish!");
            }
        }
    }

    void featureExtraction::imu_Handler(const sensor_msgs::msg::Imu::SharedPtr msg_in) {
        m_buf.lock();
        
        auto measurement = parseImuMessage(msg_in);
        
        calculateDeltaTime(measurement.timestamp);
        
        auto imudata = createImuData(measurement);
        
        updateImuOrientation(imudata);
        
        imuBuf.addMeas(imudata, measurement.timestamp);
        
        // only do it at the beginning
        imuInitialization(measurement.timestamp);
        
        m_buf.unlock();
    }

    void featureExtraction::visual_odom_Handler(const nav_msgs::msg::Odometry::SharedPtr visualOdometry)
    {
        m_buf.lock();
        visualOdomBuf.addMeas(visualOdometry, visualOdometry->header.stamp.sec + visualOdometry->header.stamp.nanosec*1e-9);
        m_buf.unlock();
    }

    void featureExtraction::assignTimeforPointCloud(pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_)
    {
        size_t cloud_size = laserCloudIn_ptr_->size();
        RCLCPP_DEBUG(this->get_logger(), "\n\ninput cloud size: %zu \n",cloud_size);
        pointCloudwithTime.reset(new pcl::PointCloud<point_os::PointcloudXYZITR>());
        pointCloudwithTime->reserve(cloud_size);

        point_os::PointcloudXYZITR point;
        for (size_t i = 0; i < cloud_size; i++)
        {
            point.x = laserCloudIn_ptr_->points[i].x;
            point.y = laserCloudIn_ptr_->points[i].y;
            point.z = laserCloudIn_ptr_->points[i].z;
            point.intensity = laserCloudIn_ptr_->points[i].intensity;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (config_.N_SCANS == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (config_.N_SCANS - 1) || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else if (config_.N_SCANS == 32)
            {
                scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
                if (scanID > (config_.N_SCANS - 1) || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else if (config_.N_SCANS == 64)
            {
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = config_.N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                // use [0 50]  > 50 remove outlies
                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
                // ROS_BREAK();
            }

            point.ring = scanID;
            float rel_time = (columnTime * int(i / config_.N_SCANS) + laserTime * (i % config_.N_SCANS)) / scanPeriod;
            float pointTime = rel_time * scanPeriod;
            point.time = pointTime;
            pointCloudwithTime->push_back(point);
        }
    }

    void featureExtraction::laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {  
        // Check if we should process this frame based on skip count
        frameCount = frameCount + 1;
        if (frameCount % config_.skipFrame != 0)
            return;

        m_buf.lock();

        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud(
            new pcl::PointCloud<point_os::PointcloudXYZITR>());
        
        tmpOusterCloudIn.reset(new pcl::PointCloud<point_os::OusterPointXYZIRT>());

        if (config_.provide_point_time)
        {

            if (config_.sensor == SensorType::VELODYNE)
            {
                pcl::fromROSMsg(*laserCloudMsg, *pointCloud);

            }
            else if (config_.sensor == SensorType::OUSTER)
            {
                // Convert to Velodyne format
                pcl::fromROSMsg(*laserCloudMsg, *tmpOusterCloudIn);
                pointCloud->points.resize(tmpOusterCloudIn->size());
                pointCloud->is_dense = tmpOusterCloudIn->is_dense;

                for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
                {
                    auto &src = tmpOusterCloudIn->points[i];
                    auto &dst = pointCloud->points[i];
                    utils::transformOusterPoints(&src, &dst, T_ouster_sensor);  // Convert the ouster points from ouster frame to sensor frame
                    dst.time = src.t * 1e-9f;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),"Unknown sensor type: %d", int(sensor));
                rclcpp::shutdown();
            }
        }
        else
        {
            pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn_ptr_);
            assignTimeforPointCloud(laserCloudIn_ptr_);
            pointCloud = pointCloudwithTime;
        }

        manageLidarBuffer(pointCloud, laserCloudMsg->header.stamp.sec + laserCloudMsg->header.stamp.nanosec * 1e-9);

        if(IMU_INIT==true or imuBuf.empty())
        {   
            undistortionAndFeatureExtraction();
            double lidar_first_time;
            lidarBuf.getFirstTime(lidar_first_time);
            lidarBuf.clean(lidar_first_time);
        }

        m_buf.unlock();
    }


    void featureExtraction::livoxHandler(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
    {   
        frameCount = frameCount + 1;
        if (frameCount % config_.skipFrame != 0)
            return; 

        m_buf.lock();
        
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud(
            new pcl::PointCloud<point_os::PointcloudXYZITR>());
        
        pointCloud->points.resize(msg->point_num);

        Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
        if (!imuBuf.empty()) {
            rotation_matrix = imu_Init->imu_laser_R_Gravity;
        } 
        
        if(config_.provide_point_time) {     
            for (uint i=0; i < msg->point_num; i++) {
                if ((msg->points[i].line < config_.N_SCANS) &&
                    ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {   
                    Eigen::Vector3d point(msg->points[i].x, msg->points[i].y, msg->points[i].z);
                    Eigen::Vector3d transformed_point = rotation_matrix * point;
                    pointCloud->points[i].x = transformed_point.x();
                    pointCloud->points[i].y = transformed_point.y();
                    pointCloud->points[i].z = transformed_point.z();
                    pointCloud->points[i].intensity = msg->points[i].reflectivity;
                    pointCloud->points[i].time = msg->points[i].offset_time / float(1000000000);
                    pointCloud->points[i].ring = msg->points[i].line;
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Please check yaml or livox driver to provide the timestamp for each point");
            rclcpp::shutdown();
        }

        manageLidarBuffer(pointCloud, msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9);

        if(IMU_INIT==true or imuBuf.empty())
        {   
            undistortionAndFeatureExtraction();
            double lidar_first_time;
            lidarBuf.getFirstTime(lidar_first_time);
            lidarBuf.clean(lidar_first_time);
        }

        m_buf.unlock();
    }

    void featureExtraction::manageLidarBuffer(
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud, 
        double timestamp)
    {
        // Check buffer size and drop oldest frames if necessary
        std::size_t curLidarBufferSize = lidarBuf.getSize();
        
        while (curLidarBufferSize >= 50) {
            double lidar_first_time;
            lidarBuf.getFirstTime(lidar_first_time);
            lidarBuf.clean(lidar_first_time);
            RCLCPP_WARN(this->get_logger(), "Lidar buffer too large, dropping frame");
            curLidarBufferSize = lidarBuf.getSize();
        }
        
        // Add measurement to buffer
        lidarBuf.addMeas(pointCloud, timestamp);
    }

} // namespace super_odometry
