//
// Unified Header for IMU Preintegration with Optional GTSAM Fixed-Lag Smoother
// Supports both traditional ISAM2 and Fixed-Lag Smoother backends
//
#pragma once
#ifndef IMUPREINTEGRATION_H
#define IMUPREINTEGRATION_H

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "utility.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

// Fixed-Lag Smoother includes from gtsam_unstable
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>

#include "super_odometry/utils/Twist.h"
#include "super_odometry/container/MapRingBuffer.h"
#include "super_odometry/config/parameter.h"
#include "super_odometry/tic_toc.h"
#include <glog/logging.h>
#include "super_odometry/sensor_data/imu/imu_data.h"

// Performance monitoring includes
#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <memory>

namespace super_odometry {

    using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
    using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
    using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
    using FrameId = std::uint64_t;

    struct imuPreintegration_config{
        float imuAccNoise;
        float imuAccBiasN;
        float imuGyrNoise;
        float imuGyrBiasN;
        float imuGravity;
        float lidar_correction_noise;
        float smooth_factor;
        bool  use_imu_roll_pitch;
        SensorType sensor;

        double imu_acc_x_limit;
        double imu_acc_y_limit;
        double imu_acc_z_limit;

        // Fixed-Lag Smoother parameters
        bool use_fixed_lag;                 // Enable fixed-lag smoother (primary mode selection)
        double fixed_lag_size;              // Lag window size in seconds
        bool use_batch_fixed_lag;           // true = BatchFixedLagSmoother, false = IncrementalFixedLagSmoother

        // Traditional ISAM2 marginalization parameters (when fixed-lag is disabled)
        bool enable_marginalization;        // Enable smart reset for traditional ISAM2
        double marginalization_window_size; // Time window in seconds for smart reset
        int marginalization_skip;           // Skip factor for marginalization
    };

    // Structure to store pose with timestamp for trajectory output
    struct TimestampedPose {
        double timestamp;
        gtsam::Pose3 worldPose;
        gtsam::Pose3 localPose;
        int graphKey;

        TimestampedPose() : timestamp(0.0), graphKey(-1) {}
        TimestampedPose(double t, const gtsam::Pose3& w, const gtsam::Pose3& l, int k) 
            : timestamp(t), worldPose(w), localPose(l), graphKey(k) {}
    };

    class imuPreintegration : public rclcpp::Node {
    public:
        imuPreintegration(const rclcpp::NodeOptions & options);
        virtual ~imuPreintegration();

        static constexpr double delta_t = 0;
        static constexpr double imu_laser_timedelay= 0.8;

        // Unified performance monitoring structure
        struct PerformanceMetrics {
            // Optimization timing
            double last_optimization_time_ms = 0.0;
            double max_optimization_time_ms = 0.0;
            double avg_optimization_time_ms = 0.0;
            int optimization_count = 0;

            // Graph update timing
            double last_graph_update_time_ms = 0.0;
            double max_graph_update_time_ms = 0.0;
            double avg_graph_update_time_ms = 0.0;

            // Graph statistics
            size_t current_graph_size = 0;
            size_t max_graph_size = 0;

            // Processing statistics
            double total_processing_time_ms = 0.0;
            int total_frames_processed = 0;

            // Memory usage
            size_t current_memory_usage_mb = 0;
            size_t peak_memory_usage_mb = 0;

            // Fixed-Lag specific metrics
            int num_marginalizations = 0;          // Number of marginalization events (Fixed-Lag)
            size_t total_marginalized_keys = 0;    // Total keys marginalized (Fixed-Lag)
            double avg_keys_per_marginalization = 0.0;

            // Traditional ISAM2 specific metrics
            int num_resets_performed = 0;          // Number of smart resets performed (Traditional)
            double time_since_last_reset = 0.0;    // Time since last reset (Traditional)

            void reset() {
                *this = PerformanceMetrics();
            }
        };

    public:
        // Core interface methods
        void initInterface();
        bool readParameters();
        void laserodometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
        void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw);
        void initial_system(double currentCorrectionTime, gtsam::Pose3 lidarPose);

        // Initialization methods for different backends
        void initializeFixedLagSmoother();
        void initializeTraditionalOptimizer();

        // Graph building methods - unified interface
        void process_imu_odometry(double currentCorrectionTime, gtsam::Pose3 lidarPose);
        bool build_graph(gtsam::Pose3 lidarPose, double curLaserodomtimestamp);

        // Backend-specific graph building methods
        bool build_graph_with_fixed_lag(gtsam::Pose3 lidarPose, double curLaserodomtimestamp);
        bool build_graph_traditional(gtsam::Pose3 lidarPose, double curLaserodomtimestamp);

        // Traditional ISAM2 methods (smart reset functionality)
        void reset_graph();
        void resetOptimization();
        void performSmartReset(int keepFromKey);
        void handleTraditionalMarginalization(double currentCorrectionTime);
        bool shouldPerformReset() const;
        void updateKeyTimestamps(double timestamp);

        // IMU processing methods
        void repropagate_imuodometry(double currentCorrectionTime);
        bool failureDetection(const gtsam::Vector3 &velCur,
                         const gtsam::imuBias::ConstantBias &biasCur);
        void integrate_imumeasurement(double currentCorrectionTime);
        void resetParams();

        // IMU initialization and preprocessing
        bool handleIMUInitialization(const sensor_msgs::msg::Imu::SharedPtr&imu_raw, 
                                    sensor_msgs::msg::Imu& thisImu);
        void initializeImu(const sensor_msgs::msg::Imu::SharedPtr& imu_raw);
        void correctLivoxGravity(sensor_msgs::msg::Imu& thisImu);
        sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu &imu_in);
        void processTiming(const sensor_msgs::msg::Imu& thisImu);

        // Publishing methods
        void updateAndPublishPath(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu);
        void publishTransform(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu);
        void prepareOdometryMessage(nav_msgs::msg::Odometry &odometry, 
                                   const sensor_msgs::msg::Imu& thisImu, 
                                   const gtsam::NavState &currentStateLocal,
                                   const gtsam::Pose3& currentWorldPose);
        void publishTransformsAndPath(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu);

        // Pose history management
        void addToPoseHistory(int key, double timestamp, const gtsam::Pose3& worldPose, const gtsam::Pose3& localPose);
        void cleanOldPoseHistory(double currentTime, double keepDuration = 60.0);
        TimestampedPose interpolatePose(double queryTime) const;

        // Performance monitoring methods
        void updatePerformanceMetrics(double opt_time_ms);
        void logPerformanceTiming(std::chrono::high_resolution_clock::time_point start_time, 
                                 double opt_time_ms, double timestamp);
        void logCurrentPerformance(double opt_time_ms);
        void reportDetailedPerformance();
        void reportPerformanceMetrics();
        void comparePerformanceWithBaseline();
        size_t getCurrentMemoryUsageMB();
        void logPerformanceToCSV(double timestamp, double opt_time_ms, double total_time_ms);

        // Utility template
        template<typename T>
        double secs(T msg) {
            return msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9;
        }

    private:
        // ROS2 interface
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubHealthStatus;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

        rclcpp::CallbackGroup::SharedPtr cb_group_;

    public:
        // GTSAM noise models
        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
        gtsam::Vector noiseModelBetweenBias;

        // IMU preintegration
        std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorOpt_;
        std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorImu_;

        // Current state estimates
        gtsam::Pose3 prevPose_;      // Previous pose in local frame
        gtsam::Vector3 prevVel_;
        gtsam::NavState prevState_;
        gtsam::imuBias::ConstantBias prevBias_;
        gtsam::NavState prevStateOdom;
        gtsam::imuBias::ConstantBias prevBiasOdom;

        // Fixed-Lag Smoother backends (when use_fixed_lag = true)
        std::unique_ptr<gtsam::BatchFixedLagSmoother> batchSmoother_;
        std::unique_ptr<gtsam::IncrementalFixedLagSmoother> incrementalSmoother_;

        // Traditional ISAM2 backend (when use_fixed_lag = false)
        gtsam::ISAM2 optimizer;
        gtsam::NonlinearFactorGraph graphFactors;
        gtsam::Values graphValues;

        // Traditional ISAM2 tracking for smart reset
        std::map<gtsam::Key, double> keyTimestamps;
        double oldestTimestamp = 0.0;
        double newestTimestamp = 0.0;
        int resetCounter = 0;

        // World pose tracking (outside of graph for relative pose formulation)
        gtsam::Pose3 worldPose;      // Current pose in world frame
        gtsam::Pose3 lastLidarPose;  // Last lidar pose for relative computation

        // Pose history for continuous trajectory output and debugging
        std::map<double, TimestampedPose> poseHistory;

        // Key tracking across resets/marginalizations
        int key = 1;
        int totalKeysProcessed = 0;  // Total keys processed across all resets
        gtsam::Pose3 accumulatedWorldTransform;  // Accumulated transform from resets

        // Performance monitoring members
        PerformanceMetrics perf_metrics_;
        bool perf_logging_enabled_ = true;
        double last_reset_timestamp_ = 0.0;
        std::chrono::high_resolution_clock::time_point start_time_;

    public:
        // Extrinsic calibration
        gtsam::Pose3 imu2cam;
        gtsam::Pose3 cam2Lidar;
        gtsam::Pose3 imu2Lidar;
        gtsam::Pose3 lidar2Imu;

    public:
        // Data buffers
        MapRingBuffer<Imu::Ptr> imuBuf;
        std::deque<sensor_msgs::msg::Imu> imuQueOpt;
        std::deque<sensor_msgs::msg::Imu> imuQueImu;
        MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> lidarOdomBuf;
        std::mutex mBuf;
        Imu::Ptr imu_Init = std::make_shared<Imu>();

    public:
        // System state flags
        bool systemInitialized = false;
        bool doneFirstOpt = false;
        bool health_status = true;
        bool imu_init_success = false;

        // IMU data
        Eigen::Quaterniond firstImu;
        Eigen::Vector3d gyr_pre;

        // Timing
        double first_imu_time_stamp;
        double last_processed_lidar_time = -1;
        double lastImuT_imu = -1;
        double lastImuT_opt = -1;
        int imuPreintegrationResetId = 0;
        int frame_count = 0;

        // System health status
        enum IMU_STATE : uint8_t {
            FAIL=0,
            SUCCESS=1,
            UNKNOW=2
        };  

        IMU_STATE RESULT;
        nav_msgs::msg::Odometry::SharedPtr cur_frame = nullptr;
        nav_msgs::msg::Odometry::SharedPtr last_frame = nullptr;
        imuPreintegration_config config_;
    };

}

#endif // IMUPREINTEGRATION_H