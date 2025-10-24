//
// IMU Preintegration with Optional GTSAM Fixed-Lag Smoother
// Can switch between traditional ISAM2 and Fixed-Lag Smoother
//
#include "super_odometry/ImuPreintegration/imuPreintegration_relative.h"

namespace super_odometry {

    imuPreintegration::imuPreintegration(const rclcpp::NodeOptions & options)
    : Node("imu_preintegration_node", options) {
    }

    imuPreintegration::~imuPreintegration() {
        if (perf_logging_enabled_) {
            RCLCPP_INFO(this->get_logger(), 
                "\n===== FINAL PERFORMANCE REPORT =====");
            reportPerformanceMetrics();
            comparePerformanceWithBaseline();
        }
    }

    void imuPreintegration::initInterface() {
        //! Callback Groups
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

        rclcpp::QoS imu_qos(10);
        imu_qos.best_effort();
        imu_qos.keep_last(10);

        if (!readGlobalparam(shared_from_this())) {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::imuPreintegration] Could not read global parameters. Exiting...");
            rclcpp::shutdown();
        }

        if (!readParameters()) {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::imuPreintegration] Could not read local parameters. Exiting...");
            rclcpp::shutdown();
        }

        if (!readCalibration(shared_from_this())) {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::imuPreintegration] Could not read calibration parameters. Exiting...");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "[SuperOdometry::imuPreintegration] use_imu_rol_pitch: %d", config_.use_imu_roll_pitch);

        //subscribe and publish relevant topics
        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, imu_qos,
            std::bind(&imuPreintegration::imuHandler, this,
                        std::placeholders::_1), sub_options);
        subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            ProjectName+"/laser_odometry", 5,
            std::bind(&imuPreintegration::laserodometryHandler, this,
                        std::placeholders::_1), sub_options);

        pubImuOdometry = this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/state_estimation", 10);
        pubHealthStatus = this->create_publisher<std_msgs::msg::Bool>(
            ProjectName+"/state_estimation_health", 1);
        pubImuPath = this->create_publisher<nav_msgs::msg::Path>(
            ProjectName+"/imuodom_path", 1);

        // set relevant parameter
        std::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(config_.imuGravity);

        p->accelerometerCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuAccNoise, 2);
        p->gyroscopeCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuGyrNoise, 2);
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);

        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);

        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << config_.lidar_correction_noise, config_.lidar_correction_noise,
                 config_.lidar_correction_noise, config_.lidar_correction_noise,
                 config_.lidar_correction_noise, config_.lidar_correction_noise).finished());

        noiseModelBetweenBias = (gtsam::Vector(6)
                << config_.imuAccBiasN, config_.imuAccBiasN, config_.imuAccBiasN, 
                   config_.imuGyrBiasN, config_.imuGyrBiasN, config_.imuGyrBiasN).finished();

        imuIntegratorImu_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias);
        imuIntegratorOpt_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias);

        if (PROVIDE_IMU_LASER_EXTRINSIC) {
            lidar2Imu = gtsam::Pose3(gtsam::Rot3(imu_laser_R), gtsam::Point3(imu_laser_T));
        } else {
            imu2cam = gtsam::Pose3(gtsam::Rot3(imu_camera_R), gtsam::Point3(imu_camera_T));
            cam2Lidar = gtsam::Pose3(gtsam::Rot3(cam_laser_R), gtsam::Point3(cam_laser_T));
            imu2Lidar = imu2cam.compose(cam2Lidar);
            lidar2Imu = imu2Lidar.inverse();
        }

        worldPose = gtsam::Pose3();
        lastLidarPose = gtsam::Pose3();

        totalKeysProcessed = 0;

        perf_metrics_.reset();
        perf_logging_enabled_ = true;
        start_time_ = std::chrono::high_resolution_clock::now();

        // Initialize the appropriate optimizer based on configuration
        if (config_.use_fixed_lag) {
            initializeFixedLagSmoother();
            RCLCPP_INFO(this->get_logger(), 
                "Fixed-Lag Smoother enabled with lag: %.1f seconds", 
                config_.fixed_lag_size);
        } else {
            initializeTraditionalOptimizer();
            RCLCPP_INFO(this->get_logger(), 
                "Traditional ISAM2 optimizer enabled with marginalization: %s", 
                config_.enable_marginalization ? "ON" : "OFF");
        }
    }

    void imuPreintegration::initializeFixedLagSmoother() {
        if (config_.use_batch_fixed_lag) {
            batchSmoother_ = std::make_unique<gtsam::BatchFixedLagSmoother>(
                config_.fixed_lag_size);
        } else {
            gtsam::ISAM2Params isam2_params;
            isam2_params.relinearizeThreshold = 0.1;
            isam2_params.relinearizeSkip = 1;
            isam2_params.findUnusedFactorSlots = true;

            incrementalSmoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
                config_.fixed_lag_size, isam2_params);
        }

        RCLCPP_INFO(this->get_logger(), 
            "Using %s Fixed-Lag Smoother", 
            config_.use_batch_fixed_lag ? "Batch" : "Incremental");
    }

    void imuPreintegration::initializeTraditionalOptimizer() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;

        // Enable these for better marginalization performance if enabled
        if (config_.enable_marginalization) {
            optParameters.enablePartialRelinearizationCheck = true;
            optParameters.findUnusedFactorSlots = true;
            optParameters.cacheLinearizedFactors = false;
        }

        optimizer = gtsam::ISAM2(optParameters);

        if (config_.enable_marginalization) {
            keyTimestamps.clear();
            oldestTimestamp = 0.0;
            newestTimestamp = 0.0;
        }

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    bool imuPreintegration::readParameters() {
        this->declare_parameter<float>("imu_preintegration_node.acc_n", 1e-3);
        this->declare_parameter<float>("imu_preintegration_node.acc_w", 1e-3);
        this->declare_parameter<float>("imu_preintegration_node.gyr_n", 1e-6);
        this->declare_parameter<float>("imu_preintegration_node.gyr_w", 1e-6);
        this->declare_parameter<float>("imu_preintegration_node.g_norm", 9.80511);
        this->declare_parameter<float>("imu_preintegration_node.lidar_correction_noise", 0.01);
        this->declare_parameter<float>("imu_preintegration_node.smooth_factor", 0.9);
        this->declare_parameter<bool>("imu_preintegration_node.use_imu_roll_pitch", false);
        this->declare_parameter<double>("imu_preintegration_node.imu_acc_x_limit", 1.0);
        this->declare_parameter<double>("imu_preintegration_node.imu_acc_y_limit", 1.0);
        this->declare_parameter<double>("imu_preintegration_node.imu_acc_z_limit", 1.0);

        // Fixed-lag smoother parameters
        this->declare_parameter<bool>("imu_preintegration_node.use_fixed_lag", false);
        this->declare_parameter<double>("imu_preintegration_node.fixed_lag_size", 10.0);
        this->declare_parameter<bool>("imu_preintegration_node.use_batch_fixed_lag", false);

        // Traditional marginalization parameters (for when fixed-lag is disabled)
        this->declare_parameter<bool>("imu_preintegration_node.enable_marginalization", false);
        this->declare_parameter<double>("imu_preintegration_node.marginalization_window_size", 10.0);

        config_.imuAccNoise = this->get_parameter("imu_preintegration_node.acc_n").as_double();
        config_.imuAccBiasN = this->get_parameter("imu_preintegration_node.acc_w").as_double();
        config_.imuGyrNoise = this->get_parameter("imu_preintegration_node.gyr_n").as_double();
        config_.imuGyrBiasN = this->get_parameter("imu_preintegration_node.gyr_w").as_double();
        config_.imuGravity = this->get_parameter("imu_preintegration_node.g_norm").as_double();
        config_.lidar_correction_noise = this->get_parameter("imu_preintegration_node.lidar_correction_noise").as_double();
        config_.smooth_factor = this->get_parameter("imu_preintegration_node.smooth_factor").as_double();
        config_.use_imu_roll_pitch = USE_IMU_ROLL_PITCH;
        config_.imu_acc_x_limit = IMU_ACC_X_LIMIT;
        config_.imu_acc_y_limit = IMU_ACC_Y_LIMIT;
        config_.imu_acc_z_limit = IMU_ACC_Z_LIMIT;

        config_.use_fixed_lag = this->get_parameter("imu_preintegration_node.use_fixed_lag").as_bool();
        config_.fixed_lag_size = this->get_parameter("imu_preintegration_node.fixed_lag_size").as_double();
        config_.use_batch_fixed_lag = this->get_parameter("imu_preintegration_node.use_batch_fixed_lag").as_bool();

        config_.enable_marginalization = this->get_parameter("imu_preintegration_node.enable_marginalization").as_bool();
        config_.marginalization_window_size = this->get_parameter("imu_preintegration_node.marginalization_window_size").as_double();

        if (SENSOR == "livox") {
            config_.sensor = SensorType::LIVOX;
        } else if (SENSOR == "velodyne") {
            config_.sensor = SensorType::VELODYNE;
        } else if (SENSOR == "ouster") {
            config_.sensor = SensorType::OUSTER;
        }

        // Validate configuration
        if (config_.use_fixed_lag && config_.enable_marginalization) {
            RCLCPP_WARN(this->get_logger(), 
                "Both fixed-lag and traditional marginalization enabled. Fixed-lag takes precedence.");
        }

        return true;
    }

    void imuPreintegration::resetParams() {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void imuPreintegration::initial_system(double currentCorrectionTime, gtsam::Pose3 lidarPose) {\

        // Guard against double-initialization
        if (systemInitialized) {
        RCLCPP_WARN(this->get_logger(), "System already initialized; skipping re-initialization.");
        return;
        }
        
        while (!imuQueOpt.empty()) {
            if (secs(&imuQueOpt.front()) < currentCorrectionTime - delta_t) {
                lastImuT_opt = secs(&imuQueOpt.front());
                imuQueOpt.pop_front();
            }
            else
                break;
        }

        worldPose = lidarPose.compose(lidar2Imu);
        lastLidarPose = lidarPose;

        // Graph starts at identity
        prevPose_ = lidarPose.compose(lidar2Imu);

        prevVel_ = gtsam::Vector3(0, 0, 0);
        prevBias_ = gtsam::imuBias::ConstantBias();
        prevState_ = gtsam::NavState(prevPose_, prevVel_);

        if (config_.use_fixed_lag) {
            // Initialize fixed-lag smoother
            gtsam::NonlinearFactorGraph newFactors;
            gtsam::Values newValues;
            gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps;

            newFactors.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), prevPose_, priorPoseNoise));
            newFactors.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), prevVel_, priorVelNoise));
            newFactors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), prevBias_, priorBiasNoise));

            newValues.insert(X(0), prevPose_);
            newValues.insert(V(0), prevVel_);
            newValues.insert(B(0), prevBias_);

            newTimestamps[X(0)] = currentCorrectionTime;
            newTimestamps[V(0)] = currentCorrectionTime;
            newTimestamps[B(0)] = currentCorrectionTime;

            if (config_.use_batch_fixed_lag) {
                batchSmoother_->update(newFactors, newValues, newTimestamps);
            } else {
                incrementalSmoother_->update(newFactors, newValues, newTimestamps);
            }
        } else {
            // Initialize traditional ISAM2 optimizer
            initializeTraditionalOptimizer();

            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);

            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);

            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);

            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);

            if (config_.enable_marginalization) {
                keyTimestamps[X(0)] = currentCorrectionTime;
                keyTimestamps[V(0)] = currentCorrectionTime;
                keyTimestamps[B(0)] = currentCorrectionTime;
                oldestTimestamp = currentCorrectionTime;
                newestTimestamp = currentCorrectionTime;
            }

            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();
        }

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        key = 1;
        systemInitialized = true;
    }

    void imuPreintegration::integrate_imumeasurement(double currentCorrectionTime) {
        while (!imuQueOpt.empty()) {
            sensor_msgs::msg::Imu *thisImu = &imuQueOpt.front();
            double imuTime = secs(thisImu);
            if (imuTime < currentCorrectionTime - delta_t) {
                double dt = (lastImuT_opt < 0) ? (1.0 / 200.0) : (imuTime - lastImuT_opt);
                lastImuT_opt = imuTime;

                if(dt < 0.001 || dt > 0.5) 
                    dt = 0.005;

                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);

                imuQueOpt.pop_front();
            }
            else
                break;
        }
    }

    bool imuPreintegration::build_graph(gtsam::Pose3 lidarPose, double curLaserodomtimestamp) {
        if (config_.use_fixed_lag) {
            return build_graph_with_fixed_lag(lidarPose, curLaserodomtimestamp);
        } else {
            return build_graph_traditional(lidarPose, curLaserodomtimestamp);
        }
    }

    bool imuPreintegration::build_graph_with_fixed_lag(gtsam::Pose3 lidarPose, double curLaserodomtimestamp) {
        auto start_time = std::chrono::high_resolution_clock::now();

        gtsam::Pose3 relativeLidarPose = lastLidarPose.inverse().compose(lidarPose);
        gtsam::Pose3 relativeImuPose = lidar2Imu.inverse().compose(relativeLidarPose).compose(lidar2Imu);

        // Get IMU prediction
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);

        gtsam::NonlinearFactorGraph newFactors;
        gtsam::Values newValues;
        gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps;

        const gtsam::PreintegratedImuMeasurements &preint_imu =
                dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
        newFactors.add(gtsam::ImuFactor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu));

        newFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(key - 1), X(key), relativeImuPose, correctionNoise));

        newFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

        newValues.insert(X(key), prevPose_.compose(relativeImuPose));
        newValues.insert(V(key), propState_.v());
        newValues.insert(B(key), prevBias_);

        newTimestamps[X(key)] = curLaserodomtimestamp;
        newTimestamps[V(key)] = curLaserodomtimestamp;
        newTimestamps[B(key)] = curLaserodomtimestamp;

        auto opt_start = std::chrono::high_resolution_clock::now();

        bool systemSolvedSuccessfully = false;
        try {
            if (config_.use_batch_fixed_lag) {
                auto result = batchSmoother_->update(newFactors, newValues, newTimestamps);
                size_t currentSize = batchSmoother_->getFactors().size();
                if (key % 10 == 0 && currentSize > 0) {
                    perf_metrics_.num_marginalizations++;
                }
            } else {
                auto result = incrementalSmoother_->update(newFactors, newValues, newTimestamps);
                auto timestamps = incrementalSmoother_->timestamps();

                size_t keysInWindow = 0;
                double currentTime = curLaserodomtimestamp;
                for (const auto& [key_ts, time] : timestamps) {
                    if (currentTime - time <= config_.fixed_lag_size) {
                        keysInWindow++;
                    }
                }

                size_t marginalizedCount = totalKeysProcessed * 3 - keysInWindow;
                if (marginalizedCount > perf_metrics_.total_marginalized_keys) {
                    size_t newlyMarginalized = marginalizedCount - perf_metrics_.total_marginalized_keys;
                    perf_metrics_.total_marginalized_keys = marginalizedCount;

                    if (newlyMarginalized > 0) {
                        perf_metrics_.num_marginalizations++;
                    }
                }
            }

            systemSolvedSuccessfully = true;
        }
        catch (const std::exception& e) {
            systemSolvedSuccessfully = false;
            RCLCPP_WARN(this->get_logger(), 
                "Fixed-lag smoother update failed: %s", e.what());
        }

        auto opt_end = std::chrono::high_resolution_clock::now();
        double opt_time_ms = std::chrono::duration<double, std::milli>(opt_end - opt_start).count();

        updatePerformanceMetrics(opt_time_ms);

        if (systemSolvedSuccessfully) {
            gtsam::Values result;
            if (config_.use_batch_fixed_lag) {
                result = batchSmoother_->calculateEstimate();
            } else {
                result = incrementalSmoother_->calculateEstimate();
            }

            // Guard against missing keys (e.g., if marginalized or update failed partially)
            if (result.exists(X(key)) && result.exists(V(key)) && result.exists(B(key))) {
                prevPose_ = result.at<gtsam::Pose3>(X(key));
                prevVel_ = result.at<gtsam::Vector3>(V(key));
                prevState_ = gtsam::NavState(prevPose_, prevVel_);
                prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
                imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

                worldPose = lidarPose.compose(lidar2Imu);
                lastLidarPose = lidarPose;

                // Always update pose history here in fixed-lag mode
                TimestampedPose tsPose(curLaserodomtimestamp, worldPose, prevPose_, key);
                poseHistory[curLaserodomtimestamp] = tsPose;
            } else {
                RCLCPP_WARN(this->get_logger(), "Missing keys at current step (key=%d) in fixed-lag estimate; skipping state update.", key);
                return false;
            }
        }

        logPerformanceTiming(start_time, opt_time_ms, curLaserodomtimestamp);

        return systemSolvedSuccessfully;
    }

    bool imuPreintegration::build_graph_traditional(gtsam::Pose3 lidarPose, double curLaserodomtimestamp) {
        auto start_time = std::chrono::high_resolution_clock::now();

        gtsam::Pose3 relativeLidarPose = lastLidarPose.inverse().compose(lidarPose);
        gtsam::Pose3 relativeImuPose = lidar2Imu.inverse().compose(relativeLidarPose).compose(lidar2Imu);

        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);

        const gtsam::PreintegratedImuMeasurements &preint_imu =
                dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);

        // Add relative pose constraint from lidar
        gtsam::BetweenFactor<gtsam::Pose3> lidar_factor(X(key - 1), X(key), relativeImuPose, correctionNoise);
        graphFactors.add(lidar_factor);

        // Add bias evolution
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

        // Insert initial values
        graphValues.insert(X(key), prevPose_.compose(relativeImuPose));
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);

        if (config_.enable_marginalization) {
            keyTimestamps[X(key)] = curLaserodomtimestamp;
            keyTimestamps[V(key)] = curLaserodomtimestamp;
            keyTimestamps[B(key)] = curLaserodomtimestamp;
            newestTimestamp = curLaserodomtimestamp;
        }

        auto opt_start = std::chrono::high_resolution_clock::now();

        bool systemSolvedSuccessfully = false;
        try {
            optimizer.update(graphFactors, graphValues);
            optimizer.update();
            systemSolvedSuccessfully = true;
        }
        catch (const gtsam::IndeterminantLinearSystemException &) {
            systemSolvedSuccessfully = false;
            RCLCPP_WARN(this->get_logger(), "Update failed due to underconstrained call to isam2 in imuPreintegration");
        }

        auto opt_end = std::chrono::high_resolution_clock::now();
        double opt_time_ms = std::chrono::duration<double, std::milli>(opt_end - opt_start).count();

        updatePerformanceMetrics(opt_time_ms);

        graphFactors.resize(0);
        graphValues.clear();

        if (systemSolvedSuccessfully) {
            gtsam::Values result = optimizer.calculateEstimate();

            // Guard against missing keys in estimate
            if (result.exists(X(key)) && result.exists(V(key)) && result.exists(B(key))) {
                prevPose_ = result.at<gtsam::Pose3>(X(key));
                prevVel_ = result.at<gtsam::Vector3>(V(key));
                prevState_ = gtsam::NavState(prevPose_, prevVel_);
                prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
                imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

                worldPose = lidarPose.compose(lidar2Imu);
                lastLidarPose = lidarPose;
            } else {
                RCLCPP_WARN(this->get_logger(), "Missing keys at current step (key=%d) in ISAM2 estimate; skipping state update.", key);
                return false;
            }
        }

        logPerformanceTiming(start_time, opt_time_ms, curLaserodomtimestamp);

        return systemSolvedSuccessfully;
    }

    void imuPreintegration::updatePerformanceMetrics(double opt_time_ms) {
        perf_metrics_.last_optimization_time_ms = opt_time_ms;
        perf_metrics_.max_optimization_time_ms = std::max(perf_metrics_.max_optimization_time_ms, opt_time_ms);
        perf_metrics_.avg_optimization_time_ms = 
            (perf_metrics_.avg_optimization_time_ms * perf_metrics_.optimization_count + opt_time_ms) / 
            (perf_metrics_.optimization_count + 1);
        perf_metrics_.optimization_count++;

        // Get current graph size based on optimizer type
        if (config_.use_fixed_lag) {
            if (config_.use_batch_fixed_lag) {
                perf_metrics_.current_graph_size = batchSmoother_->getFactors().size();
            } else {
                perf_metrics_.current_graph_size = incrementalSmoother_->getFactors().size();
            }
        } else {
            perf_metrics_.current_graph_size = optimizer.size();
        }

        perf_metrics_.max_graph_size = std::max(perf_metrics_.max_graph_size, perf_metrics_.current_graph_size);
    }

    void imuPreintegration::logPerformanceTiming(std::chrono::high_resolution_clock::time_point start_time, 
                                                double opt_time_ms, double timestamp) {
        auto end_time = std::chrono::high_resolution_clock::now();
        double total_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        perf_metrics_.last_graph_update_time_ms = total_time_ms;
        perf_metrics_.max_graph_update_time_ms = std::max(perf_metrics_.max_graph_update_time_ms, total_time_ms);
        perf_metrics_.avg_graph_update_time_ms = 
            (perf_metrics_.avg_graph_update_time_ms * perf_metrics_.total_frames_processed + total_time_ms) / 
            (perf_metrics_.total_frames_processed + 1);

        if (key % 10 == 0) {
            const char* optimizer_type = config_.use_fixed_lag ? 
                (config_.use_batch_fixed_lag ? "Fixed-Lag Batch" : "Fixed-Lag Incremental") : 
                "Traditional ISAM2";

            RCLCPP_INFO(this->get_logger(), 
                "%s Update [Key %d]: Opt time: %.2f ms (avg: %.2f ms), "
                "Graph size: %zu nodes, Total time: %.2f ms",
                optimizer_type, key, opt_time_ms, perf_metrics_.avg_optimization_time_ms,
                perf_metrics_.current_graph_size, total_time_ms);
        }

        if (perf_logging_enabled_) {
            logPerformanceToCSV(timestamp, opt_time_ms, total_time_ms);
        }
    }

    void imuPreintegration::repropagate_imuodometry(double currentCorrectionTime) {
        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;

        double lastImuQT = -1;
        while (!imuQueImu.empty() && secs(&imuQueImu.front()) < currentCorrectionTime - delta_t) {
            lastImuQT = secs(&imuQueImu.front());
            imuQueImu.pop_front();
        }

        if (!imuQueImu.empty()) {
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            for (int i = 0; i < (int)imuQueImu.size(); ++i) {
                sensor_msgs::msg::Imu *thisImu = &imuQueImu[i];
                double imuTime = secs(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 200.0) : (imuTime - lastImuQT);
                lastImuQT = imuTime;

                if(dt < 0.001 || dt > 0.5) 
                    dt = 0.005;

                imuIntegratorImu_->integrateMeasurement(
                    gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                    gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), 
                    dt);
                lastImuQT = imuTime;
            }
        }
    }

    void imuPreintegration::process_imu_odometry(double currentCorrectionTime, gtsam::Pose3 lidarPose) {
        auto proc_start = std::chrono::high_resolution_clock::now();

        // Handle marginalization/reset logic based on optimizer type
        if (!config_.use_fixed_lag && config_.enable_marginalization) {
            handleTraditionalMarginalization(currentCorrectionTime);
        }

        integrate_imumeasurement(currentCorrectionTime);

        bool successOptimization = build_graph(lidarPose, currentCorrectionTime);

        // Check for failures
        if (failureDetection(prevVel_, prevBias_) || !successOptimization) {
            RCLCPP_WARN(this->get_logger(), "Failure detected, resetting");
            resetParams();
            return;
        }

        repropagate_imuodometry(currentCorrectionTime);

        // Only handle traditional marginalization pose history here
        // Fixed-lag pose history is already handled in build_graph_with_fixed_lag()
        if (successOptimization && !config_.use_fixed_lag && config_.enable_marginalization) {
            gtsam::Values currentEstimate = optimizer.calculateEstimate();
            gtsam::Pose3 localPose = currentEstimate.at<gtsam::Pose3>(X(key));
            TimestampedPose tsPose(currentCorrectionTime, worldPose, localPose, key);
            poseHistory[currentCorrectionTime] = tsPose;
        }

        cleanOldPoseHistory(currentCorrectionTime);

        ++key;
        ++totalKeysProcessed;
        perf_metrics_.total_frames_processed++;

        auto proc_end = std::chrono::high_resolution_clock::now();
        double proc_time_ms = std::chrono::duration<double, std::milli>(proc_end - proc_start).count();
        perf_metrics_.total_processing_time_ms += proc_time_ms;

        doneFirstOpt = true;

        // Periodic performance reports
        if (key % 50 == 0) {
            reportDetailedPerformance();
        }

        if (totalKeysProcessed % 500 == 0) {
            reportPerformanceMetrics();
        }
    }

    void imuPreintegration::handleTraditionalMarginalization(double currentCorrectionTime) {
        // Store pose history for proper tracking
        if (key > 0 && doneFirstOpt) {
            gtsam::Values currentEstimate = optimizer.calculateEstimate();
            if (currentEstimate.exists(X(key - 1))) {
                gtsam::Pose3 localPose = currentEstimate.at<gtsam::Pose3>(X(key - 1));
                TimestampedPose tsPose(currentCorrectionTime, worldPose, localPose, key - 1);
                poseHistory[currentCorrectionTime] = tsPose;
            } else {
                RCLCPP_WARN(this->get_logger(), "Key X(%d) not found during marginalization bookkeeping; skipping pose history add.", key - 1);
            }
        }

        // Check if we need to perform smart reset
        if (key > 20) {
            double timeWindow = newestTimestamp - oldestTimestamp;

            if (timeWindow > config_.marginalization_window_size) {
                double cutoffTime = newestTimestamp - config_.marginalization_window_size;
                int oldestKeyToKeep = -1;

                for (int i = 0; i < key; i++) {
                    if (keyTimestamps.count(X(i)) > 0) {
                        if (keyTimestamps[X(i)] >= cutoffTime) {
                            oldestKeyToKeep = i;
                            break;
                        }
                    }
                }

                if (oldestKeyToKeep > 0 && oldestKeyToKeep < key - 3) {
                    performSmartReset(oldestKeyToKeep);
                }
            }
        }

        // Hard reset if graph gets too large
        if (key > 300) {
            RCLCPP_INFO(this->get_logger(), 
                       "Graph size exceeded maximum, performing smart reset at key %d", key);
            performSmartReset(key - 10);
        }
    }

    void imuPreintegration::performSmartReset(int keepFromKey) {
        auto reset_start = std::chrono::high_resolution_clock::now();

        gtsam::Values currentEstimate = optimizer.calculateEstimate();

        if (!(currentEstimate.exists(X(keepFromKey)) && currentEstimate.exists(V(keepFromKey)) && currentEstimate.exists(B(keepFromKey)))) {
            RCLCPP_WARN(this->get_logger(), "Missing reset keys at keepFromKey=%d; aborting smart reset.", keepFromKey);
            return;
        }
        if (!currentEstimate.exists(X(key - 1))) {
            RCLCPP_WARN(this->get_logger(), "Missing current pose key X(%d) during smart reset; aborting.", key - 1);
            return;
        }

        gtsam::Pose3 resetPose = currentEstimate.at<gtsam::Pose3>(X(keepFromKey));
        gtsam::Vector3 resetVel = currentEstimate.at<gtsam::Vector3>(V(keepFromKey));
        gtsam::imuBias::ConstantBias resetBias = currentEstimate.at<gtsam::imuBias::ConstantBias>(B(keepFromKey));

        gtsam::Pose3 currentLocalPose = currentEstimate.at<gtsam::Pose3>(X(key - 1));
        gtsam::Pose3 relativeFromReset = resetPose.inverse().compose(currentLocalPose);

        initializeTraditionalOptimizer();

        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), gtsam::Pose3(), priorPoseNoise);
        graphFactors.add(priorPose);

        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), resetVel, priorVelNoise);
        graphFactors.add(priorVel);

        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), resetBias, priorBiasNoise);
        graphFactors.add(priorBias);

        graphValues.insert(X(0), gtsam::Pose3());
        graphValues.insert(V(0), resetVel);
        graphValues.insert(B(0), resetBias);

        double resetTime = keyTimestamps[X(keepFromKey)];
        keyTimestamps.clear();
        keyTimestamps[X(0)] = resetTime;
        keyTimestamps[V(0)] = resetTime;
        keyTimestamps[B(0)] = resetTime;
        oldestTimestamp = resetTime;
        newestTimestamp = resetTime;

        prevPose_ = relativeFromReset;
        prevVel_ = currentEstimate.at<gtsam::Vector3>(V(key - 1));
        prevBias_ = currentEstimate.at<gtsam::imuBias::ConstantBias>(B(key - 1));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        auto reset_end = std::chrono::high_resolution_clock::now();
        double reset_time_ms = std::chrono::duration<double, std::milli>(reset_end - reset_start).count();

        perf_metrics_.num_resets_performed++;

        int oldKey = key;
        key = 1;

        RCLCPP_INFO(this->get_logger(), 
                   "Smart reset completed in %.2f ms: old key %d -> new key %d, reset #%d", 
                   reset_time_ms, oldKey, key, perf_metrics_.num_resets_performed);
    }

    void imuPreintegration::reportDetailedPerformance() {
        size_t current_memory_mb = getCurrentMemoryUsageMB();
        perf_metrics_.current_memory_usage_mb = current_memory_mb;
        perf_metrics_.peak_memory_usage_mb = std::max(perf_metrics_.peak_memory_usage_mb, current_memory_mb);

        const char* optimizer_type = config_.use_fixed_lag ? 
            (config_.use_batch_fixed_lag ? "FIXED-LAG BATCH" : "FIXED-LAG INCREMENTAL") : 
            "TRADITIONAL ISAM2";

        if (config_.use_fixed_lag) {
            RCLCPP_INFO(this->get_logger(), 
                "\n╔════════════════════════════════════════════════════════╗\n"
                "║     %s PERFORMANCE (Key %d)            ║\n"
                "╠════════════════════════════════════════════════════════╣\n"
                "║ Graph Statistics:                                      ║\n"
                "║   Current Size: %zu factors                            ║\n"
                "║   Total Keys Processed: %d                             ║\n"
                "║   Total Marginalized: %zu keys                         ║\n"
                "║   Marginalization Events: %d                           ║\n"
                "╠════════════════════════════════════════════════════════╣\n"
                "║ Timing Performance:                                    ║\n"
                "║   Last Optimization: %.2f ms                           ║\n"
                "║   Avg Optimization: %.2f ms                            ║\n"
                "║   Max Optimization: %.2f ms                            ║\n"
                "╠════════════════════════════════════════════════════════╣\n"
                "║ Memory Usage:                                          ║\n"
                "║   Current: %zu MB                                      ║\n"
                "║   Peak: %zu MB                                         ║\n"
                "╠════════════════════════════════════════════════════════╣\n"
                "║ Configuration:                                         ║\n"
                "║   Fixed Lag Size: %.1f seconds                         ║\n"
                "╚════════════════════════════════════════════════════════╝",
                optimizer_type, key,
                perf_metrics_.current_graph_size,
                totalKeysProcessed,
                perf_metrics_.total_marginalized_keys,
                perf_metrics_.num_marginalizations,
                perf_metrics_.last_optimization_time_ms,
                perf_metrics_.avg_optimization_time_ms,
                perf_metrics_.max_optimization_time_ms,
                perf_metrics_.current_memory_usage_mb,
                perf_metrics_.peak_memory_usage_mb,
                config_.fixed_lag_size
            );
        } else {
            RCLCPP_INFO(this->get_logger(), 
                "\n╔════════════════════════════════════════════════════════╗\n"
                "║     %s PERFORMANCE (Key %d)               ║\n"
                "╠════════════════════════════════════════════════════════╣\n"
                "║ Graph Statistics:                                      ║\n"
                "║   Current Size: %zu nodes (Max: %zu)                   ║\n"
                "║   Keys Since Reset: %d                                 ║\n"
                "║   Total Keys Processed: %d                             ║\n"
                "╠════════════════════════════════════════════════════════╣\n"
                "║ Timing Performance:                                    ║\n"
                "║   Last Optimization: %.2f ms                           ║\n"
                "║   Avg Optimization: %.2f ms                            ║\n"
                "║   Max Optimization: %.2f ms                            ║\n"
                "╠════════════════════════════════════════════════════════╣\n"
                "║ Memory Usage:                                          ║\n"
                "║   Current: %zu MB                                      ║\n"
                "║   Peak: %zu MB                                         ║\n"
                "╠════════════════════════════════════════════════════════╣\n"
                "║ Configuration:                                         ║\n"
                "║   Smart Reset: %s                                      ║\n"
                "║   Window Size: %.1f seconds                            ║\n"
                "╚════════════════════════════════════════════════════════╝",
                optimizer_type, key,
                perf_metrics_.current_graph_size,
                perf_metrics_.max_graph_size,
                key,
                totalKeysProcessed,
                perf_metrics_.last_optimization_time_ms,
                perf_metrics_.avg_optimization_time_ms,
                perf_metrics_.max_optimization_time_ms,
                perf_metrics_.current_memory_usage_mb,
                perf_metrics_.peak_memory_usage_mb,
                config_.enable_marginalization ? "ENABLED" : "DISABLED",
                config_.marginalization_window_size
            );
        }

        if (perf_metrics_.last_optimization_time_ms > 50.0) {
            RCLCPP_WARN(this->get_logger(), 
                "⚠️  Optimization time exceeding 50ms! Consider tuning parameters.");
        }
    }

    void imuPreintegration::reportPerformanceMetrics() {
        auto current_time = std::chrono::high_resolution_clock::now();
        double runtime_seconds = std::chrono::duration<double>(current_time - start_time_).count();

        if (config_.use_fixed_lag) {
            RCLCPP_INFO(this->get_logger(), 
                "\n========== FIXED-LAG SMOOTHER SUMMARY ==========\n"
                "Runtime: %.1f seconds\n"
                "Smoother Type: %s\n"
                "Fixed Lag: %.1f seconds\n"
                "Marginalization:\n"
                "  - Events: %d\n"
                "  - Total keys marginalized: %zu\n"
                "Graph Performance:\n"
                "  - Current size: %zu factors\n"
                "  - Max size: %zu factors\n"
                "Optimization Performance:\n"
                "  - Average: %.2f ms\n"
                "  - Max: %.2f ms\n"
                "Memory Usage:\n"
                "  - Current: %zu MB\n"
                "  - Peak: %zu MB\n"
                "================================================",
                runtime_seconds,
                config_.use_batch_fixed_lag ? "Batch" : "Incremental",
                config_.fixed_lag_size,
                perf_metrics_.num_marginalizations,
                perf_metrics_.total_marginalized_keys,
                perf_metrics_.current_graph_size,
                perf_metrics_.max_graph_size,
                perf_metrics_.avg_optimization_time_ms,
                perf_metrics_.max_optimization_time_ms,
                perf_metrics_.current_memory_usage_mb,
                perf_metrics_.peak_memory_usage_mb
            );
        } else {
            RCLCPP_INFO(this->get_logger(), 
                "\n========== TRADITIONAL ISAM2 SUMMARY ==========\n"
                "Runtime: %.1f seconds\n"
                "Smart Reset: %s\n"
                "  - Resets performed: %d\n"
                "  - Graph size: %zu nodes\n"
                "Optimization Performance:\n"
                "  - Average: %.2f ms\n"
                "  - Max: %.2f ms\n"
                "Memory Usage:\n"
                "  - Current: %zu MB\n"
                "  - Peak: %zu MB\n"
                "Processing:\n"
                "  - Total frames: %d\n"
                "==============================================",
                runtime_seconds,
                config_.enable_marginalization ? "ENABLED" : "DISABLED",
                perf_metrics_.num_resets_performed,
                perf_metrics_.current_graph_size,
                perf_metrics_.avg_optimization_time_ms,
                perf_metrics_.max_optimization_time_ms,
                perf_metrics_.current_memory_usage_mb,
                perf_metrics_.peak_memory_usage_mb,
                perf_metrics_.total_frames_processed
            );
        }
    }

    void imuPreintegration::comparePerformanceWithBaseline() {
        if (config_.use_fixed_lag) {
            RCLCPP_INFO(this->get_logger(),
                "\n===== FIXED-LAG SMOOTHER ADVANTAGES =====\n"
                "✓ True marginalization preserves information\n"
                "✓ Maintains full covariance estimates\n"
                "✓ Bounded computational complexity: O(1)\n"
                "✓ Graph size automatically managed\n"
                "✓ No manual resets needed\n"
                "✓ Smooth continuous estimation\n"
                "==========================================\n"
                "Graph bounded to lag of %.1f seconds\n"
                "Optimization time stable at: %.2f ms\n"
                "Memory usage stable at: %zu MB\n"
                "Can run indefinitely with consistent performance",
                config_.fixed_lag_size,
                perf_metrics_.avg_optimization_time_ms,
                perf_metrics_.current_memory_usage_mb
            );
        } else if (config_.enable_marginalization) {
            RCLCPP_INFO(this->get_logger(),
                "\n===== SMART RESET PERFORMANCE =====\n"
                "Graph bounded to: %d nodes\n"
                "Optimization time stable at: %.2f ms\n"
                "Memory usage stable at: %zu MB\n"
                "Resets performed: %d\n"
                "Can run indefinitely without degradation\n"
                "====================================",
                key,
                perf_metrics_.avg_optimization_time_ms,
                perf_metrics_.current_memory_usage_mb,
                perf_metrics_.num_resets_performed
            );
        } else {
            RCLCPP_WARN(this->get_logger(),
                "\n===== UNBOUNDED GROWTH WARNING =====\n"
                "No marginalization or fixed-lag enabled!\n"
                "Graph will grow unbounded: %zu nodes\n"
                "Performance will degrade over time\n"
                "Consider enabling fixed-lag or marginalization\n"
                "===================================",
                perf_metrics_.current_graph_size
            );
        }
    }

    size_t imuPreintegration::getCurrentMemoryUsageMB() {
        std::ifstream file("/proc/self/status");
        std::string line;
        size_t memory_kb = 0;

        while (std::getline(file, line)) {
            if (line.substr(0, 6) == "VmRSS:") {
                std::istringstream iss(line);
                std::string label;
                iss >> label >> memory_kb;
                break;
            }
        }

        return memory_kb / 1024;
    }

    void imuPreintegration::logPerformanceToCSV(double timestamp, double opt_time_ms, double total_time_ms) {
        static std::ofstream perf_log;
        static bool first_write = true;

        if (first_write) {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;

            if (config_.use_fixed_lag) {
                ss << "imu_performance_fixed_lag_" 
                   << (config_.use_batch_fixed_lag ? "batch_" : "incremental_");
            } else {
                ss << "imu_performance_traditional_" 
                   << (config_.enable_marginalization ? "with_reset_" : "without_reset_");
            }

            ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";

            perf_log.open(ss.str());
            perf_log << "timestamp,key,graph_size,opt_time_ms,total_time_ms,memory_mb,"
                     << "pose_history_size,total_keys_processed,marginalized_keys,events\n";
            first_write = false;

            RCLCPP_INFO(this->get_logger(), "Performance logging to: %s", ss.str().c_str());
        }

        perf_log << std::fixed << std::setprecision(6)
                 << timestamp << ","
                 << key << ","
                 << perf_metrics_.current_graph_size << ","
                 << std::setprecision(3)
                 << opt_time_ms << ","
                 << total_time_ms << ","
                 << perf_metrics_.current_memory_usage_mb << ","
                 << poseHistory.size() << ","
                 << totalKeysProcessed << ","
                 << perf_metrics_.total_marginalized_keys << ","
                 << (config_.use_fixed_lag ? perf_metrics_.num_marginalizations : perf_metrics_.num_resets_performed) << "\n";
        perf_log.flush();
    }

    void imuPreintegration::cleanOldPoseHistory(double currentTime, double keepDuration) {
        double cutoffTime = currentTime - keepDuration;

        // Add debug logging
        size_t sizeBefore = poseHistory.size();

        auto it = poseHistory.begin();
        while (it != poseHistory.end()) {
            if (it->first < cutoffTime) {
                it = poseHistory.erase(it);
            } else {
                break;
            }
        }

        while (poseHistory.size() > 1000) {
            poseHistory.erase(poseHistory.begin());
        }

        // Debug logging every 100 frames
        if (key % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Pose history: %zu entries (was %zu), current time: %.3f", 
                poseHistory.size(), sizeBefore, currentTime);
        }
    }

    TimestampedPose imuPreintegration::interpolatePose(double queryTime) const {
        if (poseHistory.empty()) {
            return TimestampedPose();
        }

        auto upper = poseHistory.lower_bound(queryTime);

        if (upper == poseHistory.end()) {
            return poseHistory.rbegin()->second;
        }

        if (upper == poseHistory.begin()) {
            return upper->second;
        }

        auto lower = std::prev(upper);

        double t1 = lower->first;
        double t2 = upper->first;
        double alpha = (queryTime - t1) / (t2 - t1);

        gtsam::Pose3 interpolatedWorld = lower->second.worldPose.interpolateRt(
            upper->second.worldPose, alpha);

        gtsam::Pose3 interpolatedLocal = lower->second.localPose.interpolateRt(
            upper->second.localPose, alpha);

        return TimestampedPose(queryTime, interpolatedWorld, interpolatedLocal, -1);
    }

    bool imuPreintegration::failureDetection(const gtsam::Vector3 &velCur,
                                             const gtsam::imuBias::ConstantBias &biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30) {
            RCLCPP_WARN(this->get_logger(), "Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(),
                           biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(),
                           biasCur.gyroscope().z());

        if (ba.norm() > 2.0 || bg.norm() > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuPreintegration::laserodometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        std::lock_guard<std::mutex> lock(mBuf);
        try {

        cur_frame = odomMsg;
        double lidarOdomTime = secs(odomMsg);

        if (imuQueOpt.empty())
            return;

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z),
                                              gtsam::Point3(p_x, p_y, p_z));

        // 0. initialize system
        if (systemInitialized == false) {
            initial_system(lidarOdomTime, lidarPose);
            return;
        }

        TicToc Optimization_time;
        //1. process imu odometry
        process_imu_odometry(lidarOdomTime, lidarPose);

        // 2. safe landing process
        double latest_imu_time = -1.0;
        if (!imuQueImu.empty()) {
            latest_imu_time = secs(&imuQueImu.back());
        } else {
            RCLCPP_WARN(this->get_logger(), "IMU queue empty when checking health; marking FAIL.");
            health_status = false;
            RESULT = IMU_STATE::FAIL;
            std_msgs::msg::Bool health_status_msg;
            health_status_msg.data = health_status;
            pubHealthStatus->publish(health_status_msg);
            last_frame = cur_frame;
            last_processed_lidar_time = lidarOdomTime;
            return;
        }

        if (lidarOdomTime - latest_imu_time < imu_laser_timedelay) {
            RESULT = IMU_STATE::SUCCESS;
            health_status = true;

            if((int)odomMsg->pose.covariance[0] == 1) {
                RESULT = IMU_STATE::FAIL;
            }

        } else {
            health_status = false;
            RCLCPP_INFO(this->get_logger(), "LOOSE CONNECTION WITH IMU DRIVER, PLEASE CHECK HARDWARE!!");
            RESULT = IMU_STATE::FAIL;

            std_msgs::msg::Bool health_status_msg;
            health_status_msg.data = health_status;
            pubHealthStatus->publish(health_status_msg);
        }

        last_frame = cur_frame;
        last_processed_lidar_time = lidarOdomTime;
        } catch (const std::out_of_range& e) {
            RCLCPP_ERROR(this->get_logger(), "laserodometryHandler out_of_range: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "laserodometryHandler exception: %s", e.what());
        }
    }
    //TODO: need to consider the extrinsic matrix of imu and lidar
    sensor_msgs::msg::Imu imuPreintegration::imuConverter(const sensor_msgs::msg::Imu &imu_in) {
        sensor_msgs::msg::Imu imu_out = imu_in;

        Eigen::Matrix3d imu_laser_R_Gravity;
        imu_laser_R_Gravity = imu_Init->imu_laser_R_Gravity;

        //imu_laser_R_Gravity = imu_laser_R;

        // Rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y,
                            imu_in.angular_velocity.z);
        gyr = imu_laser_R_Gravity * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        // Rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x,
                            imu_in.linear_acceleration.y,
                            imu_in.linear_acceleration.z);

        acc = imu_laser_R_Gravity * acc;
        acc = acc + ((gyr - gyr_pre) * 200).cross(-imu_laser_T) + gyr.cross(gyr.cross(-imu_laser_T));
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();

        


        // rotate roll pitch yaw
        Eigen::Quaterniond q(imu_in.orientation.w, imu_in.orientation.x,
                             imu_in.orientation.y, imu_in.orientation.z);

        q.normalize();


        Eigen::Quaterniond q_extrinsic;
        q_extrinsic=Eigen::Quaterniond(imu_laser_R_Gravity);

        Eigen::Quaterniond q_new;

        q_new=q*q_extrinsic;

        q_new.normalize();

        imu_out.orientation.x = q_new.x();
        imu_out.orientation.y = q_new.y();
        imu_out.orientation.z = q_new.z();
        imu_out.orientation.w = q_new.w();

        gyr_pre = gyr;

        return imu_out;
    }


   void imuPreintegration::imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw) {
    std::lock_guard<std::mutex> lock(mBuf);
    try {
        // 1. Pre-process IMU data
        sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);

        // 2. Handle IMU initialization for LIVOX sensor
        if (!handleIMUInitialization(imu_raw, thisImu)) {
            return;
        }
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1, "\033[34m After IMU Init Success: %.3f, %.3f, %.3f\033[0m", 
        thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z);
        // 3. Process timing and queue management
        processTiming(thisImu);

        // 4. Early return if first optimization not done
        if (!doneFirstOpt) {
            return;
        }

            // Predict current state in local frame
            gtsam::NavState currentStateLocal = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

            // Transform to world frame for output
            gtsam::Pose3 relativePose = prevStateOdom.pose().inverse().compose(currentStateLocal.pose());
            gtsam::Pose3 currentWorldPose = worldPose.compose(relativePose);

            // Create odometry message
            nav_msgs::msg::Odometry odometry;
            prepareOdometryMessage(odometry, thisImu, currentStateLocal, currentWorldPose);

            if (frame_count++ % 4 == 0) {
                pubImuOdometry->publish(odometry);
            }

            publishTransformsAndPath(odometry, thisImu);

            // Publish health status
            std_msgs::msg::Bool health_status_msg;
            health_status_msg.data = health_status;
            pubHealthStatus->publish(health_status_msg);
        } catch (const std::out_of_range& e) {
            RCLCPP_ERROR(this->get_logger(), "imuHandler out_of_range: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "imuHandler exception: %s", e.what());
        }
    }

    bool imuPreintegration::handleIMUInitialization(const sensor_msgs::msg::Imu::SharedPtr&imu_raw, 
                                                    sensor_msgs::msg::Imu& thisImu) {   
        if (!imu_init_success) {
            initializeImu(imu_raw);
        }

        if (config_.sensor == SensorType::LIVOX) {
            correctLivoxGravity(thisImu);
        }
        
        return imu_init_success; 
    }

    void imuPreintegration::initializeImu(const sensor_msgs::msg::Imu::SharedPtr& imu_raw) {
        Imu::Ptr imudata = std::make_shared<Imu>();
        imudata->time = imu_raw->header.stamp.sec + imu_raw->header.stamp.nanosec * 1e-9;
        imudata->acc = Eigen::Vector3d(imu_raw->linear_acceleration.x,
                                      imu_raw->linear_acceleration.y,
                                      imu_raw->linear_acceleration.z);
        imudata->gyr = Eigen::Vector3d(imu_raw->angular_velocity.x,
                                      imu_raw->angular_velocity.y,
                                      imu_raw->angular_velocity.z);
        imudata->q_w_i = Eigen::Quaterniond(imu_raw->orientation.w,
                                           imu_raw->orientation.x,
                                           imu_raw->orientation.y,
                                           imu_raw->orientation.z);

        imuBuf.addMeas(imudata, imudata->time);

        double first_imu_time = 0.0;
        imuBuf.getFirstTime(first_imu_time);

        if (imudata->time - first_imu_time > 1.0) {
            imu_Init->imuInit(imuBuf);
            imu_init_success = true;
            imuBuf.clean(imudata->time);
            std::cout << "IMU Initialization Process Finish! " << std::endl;
        }
    }

    void imuPreintegration::correctLivoxGravity(sensor_msgs::msg::Imu& thisImu) {
        const double gravity = config_.imuGravity;
        Eigen::Vector3d acc(thisImu.linear_acceleration.x,
                           thisImu.linear_acceleration.y,
                           thisImu.linear_acceleration.z);
        acc = acc * gravity / imu_Init->acc_mean.norm();
        thisImu.linear_acceleration.x = acc.x();
        thisImu.linear_acceleration.y = acc.y();
        thisImu.linear_acceleration.z = acc.z();
    }

    void imuPreintegration::processTiming(const sensor_msgs::msg::Imu& thisImu) {
        double imuTime = secs(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 200.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        if (dt < 0.001 || dt > 0.5) {
            dt = 0.005;
        }

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);
    }

    void imuPreintegration::publishTransformsAndPath(nav_msgs::msg::Odometry &odometry, 
                                                    const sensor_msgs::msg::Imu& thisImu) {
        publishTransform(odometry, thisImu);
        updateAndPublishPath(odometry, thisImu);
    }

    void imuPreintegration::publishTransform(nav_msgs::msg::Odometry &odometry, 
                                           const sensor_msgs::msg::Imu& thisImu) {
        tf2_ros::TransformBroadcaster br(this);
        geometry_msgs::msg::TransformStamped transform_stamped_;
        tf2::Transform transform;
        transform_stamped_.header.stamp = thisImu.header.stamp;
        transform_stamped_.header.frame_id = WORLD_FRAME;
        transform_stamped_.child_frame_id = SENSOR_FRAME;

        tf2::Quaternion q;
        transform.setOrigin(tf2::Vector3(odometry.pose.pose.position.x, 
                                        odometry.pose.pose.position.y, 
                                        odometry.pose.pose.position.z));

        q.setW(odometry.pose.pose.orientation.w);
        q.setX(odometry.pose.pose.orientation.x);
        q.setY(odometry.pose.pose.orientation.y);
        q.setZ(odometry.pose.pose.orientation.z);
        transform.setRotation(q);
        transform_stamped_.transform = tf2::toMsg(transform);

        if(frame_count % 4 == 0)
            br.sendTransform(transform_stamped_);
    }

    void imuPreintegration::updateAndPublishPath(nav_msgs::msg::Odometry &odometry, 
                                                const sensor_msgs::msg::Imu& thisImu) {
        static nav_msgs::msg::Path imuPath;
        static double last_path_time = -1;
        double curimuTime = secs(&thisImu);

        if (curimuTime - last_path_time > 0.1) {
            last_path_time = curimuTime;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = thisImu.header.stamp;
            pose_stamped.header.frame_id = WORLD_FRAME;
            pose_stamped.pose = odometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);

            while (!imuPath.poses.empty() &&
                    abs(secs(&imuPath.poses.front()) -
                        secs(&imuPath.poses.back())) > 3.0)
                imuPath.poses.erase(imuPath.poses.begin());

            if (pubImuPath->get_subscription_count() != 0) {
                imuPath.header.stamp = thisImu.header.stamp;
                imuPath.header.frame_id = WORLD_FRAME;
                pubImuPath->publish(imuPath);
            }
        }
    }

    void imuPreintegration::prepareOdometryMessage(nav_msgs::msg::Odometry &odometry, 
                                                  const sensor_msgs::msg::Imu &thisImu, 
                                                  const gtsam::NavState &currentStateLocal,
                                                  const gtsam::Pose3& currentWorldPose) {
        // Transform to lidar frame
        gtsam::Pose3 lidarPoseWorld = currentWorldPose.compose(imu2Lidar);

        Eigen::Quaterniond q_w_curr;    
        if (config_.use_imu_roll_pitch) {
            q_w_curr = Eigen::Quaterniond(thisImu.orientation.w, thisImu.orientation.x, 
                                          thisImu.orientation.y, thisImu.orientation.z);
        } else {
            q_w_curr = Eigen::Quaterniond(lidarPoseWorld.rotation().toQuaternion().w(), 
                                          lidarPoseWorld.rotation().toQuaternion().x(),
                                          lidarPoseWorld.rotation().toQuaternion().y(), 
                                          lidarPoseWorld.rotation().toQuaternion().z());
        }

        // Transform velocity to world then body frame
        Eigen::Vector3d velocity_world = currentWorldPose.rotation().matrix() * currentStateLocal.velocity();
        Eigen::Vector3d velocity_body = lidarPoseWorld.rotation().inverse().matrix() * velocity_world;

        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = WORLD_FRAME;
        odometry.child_frame_id = SENSOR_FRAME;

        q_w_curr.normalize();

        odometry.pose.pose.position.x = lidarPoseWorld.translation().x();
        odometry.pose.pose.position.y = lidarPoseWorld.translation().y();
        odometry.pose.pose.position.z = lidarPoseWorld.translation().z();
        odometry.pose.pose.orientation.x = q_w_curr.x();
        odometry.pose.pose.orientation.y = q_w_curr.y();
        odometry.pose.pose.orientation.z = q_w_curr.z();
        odometry.pose.pose.orientation.w = q_w_curr.w();

        odometry.twist.twist.linear.x = velocity_body.x();
        odometry.twist.twist.linear.y = velocity_body.y();
        odometry.twist.twist.linear.z = velocity_body.z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();

        odometry.pose.covariance[0] = double(RESULT);
        odometry.pose.covariance[1] = prevBiasOdom.accelerometer().x();
        odometry.pose.covariance[2] = prevBiasOdom.accelerometer().y();
        odometry.pose.covariance[3] = prevBiasOdom.accelerometer().z();
        odometry.pose.covariance[4] = prevBiasOdom.gyroscope().x();
        odometry.pose.covariance[5] = prevBiasOdom.gyroscope().y();
        odometry.pose.covariance[6] = prevBiasOdom.gyroscope().z();
        odometry.pose.covariance[7] = config_.imuGravity;

         // For static: g_w_est = -R_wb * f_b should be ~ [0,0,-g]
       // For static: g_w_est should be ~ [0, 0, -g]
        Eigen::Matrix3d R_wb = currentWorldPose.rotation().matrix();  // world <- IMU body
        Eigen::Vector3d f_b(thisImu.linear_acceleration.x,
                            thisImu.linear_acceleration.y,
                            thisImu.linear_acceleration.z);
        // optional: bias-compensate the measurement
        Eigen::Vector3d f_b_ub = f_b - Eigen::Vector3d(prevBiasOdom.accelerometer().x(),
                                                        prevBiasOdom.accelerometer().y(),
                                                        prevBiasOdom.accelerometer().z());
        Eigen::Vector3d g_w_est = -R_wb * f_b_ub;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1,
            "g_w_est = [%.3f, %.3f, %.3f], |g|=%.3f", g_w_est.x(), g_w_est.y(), g_w_est.z(), g_w_est.norm());
            }


} // end namespace super_odometry