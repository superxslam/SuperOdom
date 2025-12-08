//
// Created by shibo zhao on 2020-09-27.
//
#include "super_odometry/ImuPreintegration/imuPreintegration_current.h"


namespace super_odometry {

    imuPreintegration::imuPreintegration(const rclcpp::NodeOptions & options)
    : Node("imu_preintegration_node", options) {
    }
    
    void imuPreintegration::initInterface() {
        //! Callback Groups
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

        rclcpp::QoS imu_qos(10);
        imu_qos.best_effort();  // Use BEST_EFFORT reliability
        imu_qos.keep_last(10);  // Keep last 10 messages

        if (!readGlobalparam(shared_from_this())) {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::imuPreintegration] Could not read global parameters. Exiting...");
            rclcpp::shutdown();
        }

        if (!readParameters()) {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::imuPreintegration] Could not read local parameters. Exiting...");
            rclcpp::shutdown();
        }

        if (!readCalibration(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::imuPreintegration] Could not read calibration parameters. Exiting...");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "[SuperOdometry::imuPreintegration] use_imu_rol_pitch:  %d", config_.use_imu_roll_pitch);

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
        
        // set relevant parameter (we'll override gravity direction in map after IMU init)
        preint_params_ = gtsam::PreintegrationParams::MakeSharedU(config_.imuGravity);
        
        preint_params_->accelerometerCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuAccNoise, 2); // acc white noise in continuous
        preint_params_->gyroscopeCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuGyrNoise, 2); // gyro white noise in continuous
        preint_params_->integrationCovariance = gtsam::Matrix33::Identity(3, 3) *
                                   pow(1e-4, 2); // error committed in integrating position from velocities

        gtsam::imuBias::ConstantBias prior_imu_bias(
                (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m

        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);                      // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6,
                                                             1e-1);                    // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, config_.lidar_correction_noise); // meter


        noiseModelBetweenBias = (gtsam::Vector(6)
                << config_.imuAccBiasN,
                config_.imuAccBiasN, config_.imuAccBiasN, config_.imuGyrBiasN, config_.imuGyrBiasN, config_.imuGyrBiasN)
                .finished();
        imuIntegratorImu_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(preint_params_, prior_imu_bias); // setting up the IMU integration for IMU message
        imuIntegratorOpt_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(preint_params_, prior_imu_bias); // setting up the IMU integration for optimization

        //set extrinsic matrix for laser and imu
        if (PROVIDE_IMU_LASER_EXTRINSIC) {
            lidar2Imu = gtsam::Pose3(gtsam::Rot3(imu_laser_R), gtsam::Point3(imu_laser_T));
            imu2Lidar = lidar2Imu.inverse();
           

        } else {
            imu2cam = gtsam::Pose3(gtsam::Rot3(imu_camera_R), gtsam::Point3(imu_camera_T));
            cam2Lidar = gtsam::Pose3(gtsam::Rot3(cam_laser_R), gtsam::Point3(cam_laser_T));
            imu2Lidar = imu2cam.compose(cam2Lidar);
            lidar2Imu = imu2Lidar.inverse();
         
        }

    }
    

    bool imuPreintegration::readParameters()
    {
        this->declare_parameter<float>("imu_preintegration_node.acc_n", 1e-3);
        this->declare_parameter<float>("imu_preintegration_node.acc_w", 1e-3);
        this->declare_parameter<float>("imu_preintegration_node.gyr_n", 1e-6);
        this->declare_parameter<float>("imu_preintegration_node.gyr_w", 1e-6);
        this->declare_parameter<float>("imu_preintegration_node.g_norm", 9.80511);
        this->declare_parameter<float>("imu_preintegration_node.lidar_correction_noise",0.01);
        this->declare_parameter<float>("imu_preintegration_node.smooth_factor",0.9);
        this->declare_parameter<double>("imu_preintegration_node.predict_future_secs", 0.005);
        this->declare_parameter<bool>("imu_preintegration_node.use_imu_roll_pitch", false);
        this->declare_parameter<double>("imu_preintegration_node.imu_acc_x_limit", 1.0);
        this->declare_parameter<double>("imu_preintegration_node.imu_acc_y_limit", 1.0);
        this->declare_parameter<double>("imu_preintegration_node.imu_acc_z_limit", 1.0);

        config_.imuAccNoise = this->get_parameter("imu_preintegration_node.acc_n").as_double();
        config_.imuAccBiasN = this->get_parameter("imu_preintegration_node.acc_w").as_double();
        config_.imuGyrNoise = this->get_parameter("imu_preintegration_node.gyr_n").as_double();
        config_.imuGyrBiasN = this->get_parameter("imu_preintegration_node.gyr_w").as_double();
        config_.imuGravity = this->get_parameter("imu_preintegration_node.g_norm").as_double();
        config_.lidar_correction_noise = this->get_parameter("imu_preintegration_node.lidar_correction_noise").as_double();
        config_.smooth_factor = this->get_parameter("imu_preintegration_node.smooth_factor").as_double();
        config_.predict_future_secs = this->get_parameter("imu_preintegration_node.predict_future_secs").as_double();
        // config_.use_imu_roll_pitch = this->get_parameter("imu_preintegration_node.use_imu_roll_pitch").as_bool();
        config_.imu_acc_x_limit = this->get_parameter("imu_preintegration_node.imu_acc_x_limit").as_double();
        config_.imu_acc_y_limit = this->get_parameter("imu_preintegration_node.imu_acc_y_limit").as_double();
        config_.imu_acc_z_limit = this->get_parameter("imu_preintegration_node.imu_acc_z_limit").as_double();
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

    void imuPreintegration::resetOptimization() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void imuPreintegration::resetParams() {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }


    void imuPreintegration::reset_graph() {

        // get updated noise before reset
        gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise, updatedVelNoise, updatedBiasNoise;

        try
        {
            updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
            updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
            updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to reset graph. No marginal covariance key");
            updatedPoseNoise = priorPoseNoise;
            updatedVelNoise = priorVelNoise;
            updatedBiasNoise = priorBiasNoise;
        }

        // reset graph
        resetOptimization();
        // add pose
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                   updatedPoseNoise);
        graphFactors.add(priorPose);
        // add velocity
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_,
                                                    updatedVelNoise);
        graphFactors.add(priorVel);
        // add bias
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
                B(0), prevBias_, updatedBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        key = 1;
    }

    void imuPreintegration::initial_system(double currentCorrectionTime, gtsam::Pose3 lidarPose) {
        resetOptimization();

    // Configure gravity in LiDAR map frame for preintegration once after IMU init
    if (!gravity_in_map_set_ && imu_init_success) {
        Eigen::Vector3d g_b_dir = -imu_Init->acc_mean;
        if (g_b_dir.norm() > 1e-6) {
            g_b_dir.normalize();
            Eigen::Matrix3d R_li = imu2Lidar.rotation().matrix(); // IMU->LiDAR
            Eigen::Vector3d g_m_dir = (R_li * g_b_dir).normalized(); // gravity in LiDAR map frame
            preint_params_->n_gravity = gtsam::Vector3(
                config_.imuGravity * g_m_dir.x(),
                config_.imuGravity * g_m_dir.y(),
                config_.imuGravity * g_m_dir.z());
            gravity_in_map_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Set preintegration gravity vector in LiDAR map frame: [%.3f, %.3f, %.3f]",
                        preint_params_->n_gravity.x(), preint_params_->n_gravity.y(), preint_params_->n_gravity.z());
                        //preint_params_->n_gravity = gtsam::Vector3(0.0, 0.0, -config_.imuGravity);

        } else {
            RCLCPP_WARN(this->get_logger(), "IMU acc_mean too small; cannot set map gravity yet.");
        }
    }

        while (!imuQueOpt.empty()) {
            if (secs(&imuQueOpt.front()) < currentCorrectionTime - delta_t) {
                lastImuT_opt = secs(&imuQueOpt.front());
                imuQueOpt.pop_front();
            }
            else
                break;
        }

        prevPose_ = lidarPose.compose(lidar2Imu);

        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                   priorPoseNoise);
        graphFactors.add(priorPose);

        prevVel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_,
                                                    priorVelNoise);
        graphFactors.add(priorVel);

        prevBias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
                B(0), prevBias_, priorBiasNoise);
        graphFactors.add(priorBias);

        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);

        try {
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();
        }
        catch (const gtsam::IndeterminantLinearSystemException &e) {
            RCLCPP_ERROR(this->get_logger(), "Initial system setup failed: %s", e.what());
            resetOptimization();
            return;
        }

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        key = 1;
        systemInitialized = true;
    }

    void imuPreintegration::integrate_imumeasurement(double currentCorrectionTime) {
        // 1. integrate imu data and optimize

        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::msg::Imu *thisImu = &imuQueOpt.front();
            double imuTime = secs(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 200.0) : (imuTime - lastImuT_opt);
                lastImuT_opt = imuTime;

                if(dt < 0.001 || dt > 0.5) 
                    dt = 0.005;
               
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);

                imuQueOpt.pop_front();
            }
            else
                break;
        }

    }


    bool imuPreintegration::build_graph(gtsam::Pose3 lidarPose, double curLaserodomtimestamp) {


        // add laser pose prior factor

        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);

        // insert predicted values
        gtsam::NavState propState_ =
                imuIntegratorOpt_->predict(prevState_, prevBias_);


        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, correctionNoise);
        graphFactors.add(pose_factor);

        const double dt_imu = imuIntegratorOpt_->deltaTij();
        const gtsam::PreintegratedImuMeasurements &preint_imu =
            dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);

        if (dt_imu >= 1e-4) {
            // Add IMU factor and bias evolution when we have meaningful IMU span
            graphFactors.add(gtsam::ImuFactor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu));
            double dt_bias = std::max(dt_imu, 1e-4);
            graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                gtsam::noiseModel::Diagonal::Sigmas(std::sqrt(dt_bias) * noiseModelBetweenBias)));

            graphValues.insert(X(key), propState_.pose());
            graphValues.insert(V(key), propState_.v());
            graphValues.insert(B(key), prevBias_);
        } else {
            // IMU span too small; add weak priors on V(key) and B(key) to avoid singularity
            auto weakVel = gtsam::noiseModel::Isotropic::Sigma(3, 1e3);
            auto weakBias = gtsam::noiseModel::Isotropic::Sigma(6, 1e3);
            graphFactors.add(gtsam::PriorFactor<gtsam::Vector3>(V(key), prevVel_, weakVel));
            graphFactors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(key), prevBias_, weakBias));
            graphValues.insert(X(key), curPose);
            graphValues.insert(V(key), prevVel_);
            graphValues.insert(B(key), prevBias_);
        }
        
  
        // optimize
        bool systemSolvedSuccessfully = false;
        try {
            optimizer.update(graphFactors, graphValues);
            optimizer.update();
            systemSolvedSuccessfully = true;
        }
        catch (const gtsam::IndeterminantLinearSystemException &e) {
            systemSolvedSuccessfully = false;
            RCLCPP_ERROR(this->get_logger(), "GTSAM IndeterminantLinearSystemException: %s", e.what());
            RCLCPP_WARN(this->get_logger(), "Resetting optimization graph due to underconstrained system");
            resetOptimization();
            resetParams();
            return false;
        }
        catch (const std::exception &e) {
            systemSolvedSuccessfully = false;
            RCLCPP_ERROR(this->get_logger(), "Optimization failed with exception: %s", e.what());
            return false;
        }

        graphFactors.resize(0);
        graphValues.clear();

        if (systemSolvedSuccessfully) {

            gtsam::Values result = optimizer.calculateEstimate();
            prevPose_ = result.at<gtsam::Pose3>(X(key));
            prevVel_ = result.at<gtsam::Vector3>(V(key));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        }        
        return systemSolvedSuccessfully;
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
                double dt = (lastImuQT < 0) ? (1.0 / 200.0) :(imuTime - lastImuQT);
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

    void imuPreintegration::process_imu_odometry(double currentCorrectionTime, gtsam::Pose3 relativePose) {

        // reset graph for speed and stability
        if (key > 100) {  // Reduced threshold for more frequent resets
            reset_graph();
        }

        // 1. integrate_imumeasurement
        integrate_imumeasurement(currentCorrectionTime);

        lidarodom_w_cur = relativePose;

        // 2. build_graph
        bool successOptimization = build_graph(lidarodom_w_cur, currentCorrectionTime);
        
        // 3. check optimization
        if (failureDetection(prevVel_, prevBias_) || !successOptimization) {
            RCLCPP_WARN(this->get_logger(), "failureDetected");
            resetParams();
            return;
        }

        // 4. reprogate_imuodometry
       repropagate_imuodometry(currentCorrectionTime);
        
        ++key;

        doneFirstOpt = true;
    }

    bool imuPreintegration::failureDetection(const gtsam::Vector3 &velCur,
                                             const gtsam::imuBias::ConstantBias &biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 100) {
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
        double latest_imu_time = secs(&imuQueImu.back());

        if (lidarOdomTime - latest_imu_time < imu_laser_timedelay) {
            RESULT = IMU_STATE::SUCCESS;
            health_status = true;
          
            if((int)odomMsg->pose.covariance[0] == 1) {
                RESULT = IMU_STATE::FAIL;
            }
         
        } else {
            health_status = false;
            if (cur_frame != nullptr && last_frame != nullptr) {
                Eigen::Vector3d velocity_curr;
                velocity_curr.x() =
                    (cur_frame->pose.pose.position.x - last_frame->pose.pose.position.x) /
                    (secs(cur_frame) - secs(last_frame));
                velocity_curr.y() = (cur_frame->pose.pose.position.y - last_frame->pose.pose.position.y) /
                                    (secs(cur_frame) - secs(last_frame));
                velocity_curr.z() = (cur_frame->pose.pose.position.z - last_frame->pose.pose.position.z) /
                                    (secs(cur_frame) - secs(last_frame));

                RCLCPP_INFO(this->get_logger(), "LOOSE CONNECTION WITH IMU DRIVER, PLEASE CHECK HARDWARE!!");
                RESULT = IMU_STATE::FAIL;
                health_status = false;
                nav_msgs::msg::Odometry odometry;

                std_msgs::msg::Bool health_status_msg;
                health_status_msg.data = health_status;
                pubHealthStatus->publish(health_status_msg);
            }
        }

        last_frame = cur_frame;
        last_processed_lidar_time = lidarOdomTime;
    }
    //TODO: need to consider the extrinsic matrix of imu and lidar
    sensor_msgs::msg::Imu imuPreintegration::imuConverter(const sensor_msgs::msg::Imu &imu_in) {

        sensor_msgs::msg::Imu imu_out = imu_in;

        Eigen::Matrix3d imu_laser_R_Gravity;
        imu_laser_R_Gravity=imu_Init->imu_laser_R_Gravity;

        imu_laser_R_Gravity=Eigen::Matrix3d::Identity(); // keep raw IMU; don't rotate into LiDAR here

        //FIXME: temporary set imu_laser_R_Gravity to identity matrix for debugging
       //mu_laser_R_Gravity = Eigen::Matrix3d::Identity();
 


        // rotate gyro and acc only when sensor is livox
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y,
                            imu_in.angular_velocity.z);
        gyr=imu_laser_R_Gravity*gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();


        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x,
                            imu_in.linear_acceleration.y,
                            imu_in.linear_acceleration.z);

        // keep raw acceleration in IMU body frame; no lever arm correction here
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


    
    // 1. Pre-process IMU data
    sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);
    
  


    // 2. Handle IMU initialization for LIVOX sensor
    if (!handleIMUInitialization(imu_raw, thisImu)) {
        return;
    }



    // 3. Process timing and queue management
    processTiming(thisImu);

    // 4. Early return if first optimization not done
    if (!doneFirstOpt) {
        return;
    }

        // 5. Prepare and publish odometry
        // Optionally lead the prediction slightly to reduce publish delay:
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
        if (config_.predict_future_secs > 0.0) {
            // Apply a small forward rotation using latest gyro (bias-compensated) assuming constant rate over lead
            const Eigen::Vector3d w_meas(thisImu.angular_velocity.x,
                                         thisImu.angular_velocity.y,
                                         thisImu.angular_velocity.z);
            Eigen::Vector3d w_unbiased = w_meas - Eigen::Vector3d(prevBiasOdom.gyroscope().x(),
                                                                  prevBiasOdom.gyroscope().y(),
                                                                  prevBiasOdom.gyroscope().z());
            // Build small-angle quaternion without Sophus dependency
            double theta = w_unbiased.norm() * config_.predict_future_secs;
            Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
            if (theta > 1e-12) {
                dq = Eigen::AngleAxisd(theta, w_unbiased.normalized());
            }
            gtsam::Rot3 R_lead(dq);
            gtsam::Pose3 pose_lead(R_lead * currentState.attitude(), currentState.position());
            currentState = gtsam::NavState(pose_lead, currentState.velocity());
        }
    nav_msgs::msg::Odometry odometry;
    publishOdometry(thisImu, currentState, odometry);
    publishTransformsAndPath(odometry,  thisImu);   
   
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
      std::cout<<"IMU Initialization Process Finish! "<<std::endl;
    }
}


void imuPreintegration::correctLivoxGravity(sensor_msgs::msg::Imu& thisImu) {

        const double gravity = config_.imuGravity;
        Eigen::Vector3d acc(thisImu.linear_acceleration.x,
                           thisImu.linear_acceleration.y,
                           thisImu.linear_acceleration.z);
        double acc_mean_norm = imu_Init->acc_mean.norm();
       // RCLCPP_INFO_STREAM(this->get_logger(), "Gravity: " << gravity << ", Acc Mean Norm: " << acc_mean_norm);
        if(acc_mean_norm>1e-6){
            acc = acc * gravity / acc_mean_norm;
        }
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



void imuPreintegration::publishOdometry(
    const sensor_msgs::msg::Imu& thisImu,
    const gtsam::NavState& currentState, nav_msgs::msg::Odometry &odometry) {
    
    prepareOdometryMessage(odometry, thisImu, currentState);
    
    if (frame_count++ % 1 == 0) {
        pubImuOdometry->publish(odometry);
    }

    // Publish health status
    std_msgs::msg::Bool health_status_msg;
    health_status_msg.data = health_status;
    pubHealthStatus->publish(health_status_msg);
}



void imuPreintegration::publishTransformsAndPath(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu) {
    publishTransform(odometry, thisImu);
    updateAndPublishPath(odometry,thisImu);
}

void imuPreintegration::publishTransform(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu){
    
    tf2_ros::TransformBroadcaster br(this);
    geometry_msgs::msg::TransformStamped transform_stamped_;
    tf2::Transform transform;
    transform_stamped_.header.stamp  = thisImu.header.stamp;
    transform_stamped_.header.frame_id = WORLD_FRAME;
    transform_stamped_.child_frame_id = SENSOR_FRAME;
    
    tf2::Quaternion q;
    transform.setOrigin(tf2::Vector3(odometry.pose.pose.position.x, 
    odometry.pose.pose.position.y, odometry.pose.pose.position.z));

    q.setW(odometry.pose.pose.orientation.w);
    q.setX(odometry.pose.pose.orientation.x);
    q.setY(odometry.pose.pose.orientation.y);
    q.setZ(odometry.pose.pose.orientation.z);
    transform.setRotation(q);
    transform_stamped_.transform = tf2::toMsg(transform);
    if(frame_count%1==0)
        br.sendTransform(transform_stamped_);
    
    //Publish the static gravity aligned tf 
    geometry_msgs::msg::TransformStamped transform_gravity_aligned;
    transform_gravity_aligned.header.stamp = thisImu.header.stamp;
    transform_gravity_aligned.header.frame_id = SENSOR_FRAME;
    transform_gravity_aligned.child_frame_id = "gravity";
    {
        Eigen::Quaterniond qS(imu_Init->imu_laser_R_Gravity);
        qS.normalize();
        transform_gravity_aligned.transform.rotation.w = qS.w();
        transform_gravity_aligned.transform.rotation.x = qS.x();
        transform_gravity_aligned.transform.rotation.y = qS.y();
        transform_gravity_aligned.transform.rotation.z = qS.z();
    }
    transform_gravity_aligned.transform.translation.x = 0.0;
    transform_gravity_aligned.transform.translation.y = 0.0;
    transform_gravity_aligned.transform.translation.z = 0.0;
    if(frame_count%1==0)
        br.sendTransform(transform_gravity_aligned);


}

void imuPreintegration::updateAndPublishPath(nav_msgs::msg::Odometry &odometry, const sensor_msgs::msg::Imu& thisImu){
    static nav_msgs::msg::Path imuPath;
    static double last_path_time = -1;
    double curimuTime = secs(&thisImu);
    if (curimuTime - last_path_time > 0.1)
    {
        last_path_time = curimuTime;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = thisImu.header.stamp;
        pose_stamped.header.frame_id = WORLD_FRAME;
        pose_stamped.pose = odometry.pose.pose;
        imuPath.poses.push_back(pose_stamped);
        while (!imuPath.poses.empty() &&
                abs(secs(&imuPath.poses.front()) -
                    secs(&imuPath.poses.back())) > 50.0)
            imuPath.poses.erase(imuPath.poses.begin());
        if (pubImuPath->get_subscription_count() != 0)
        {
            imuPath.header.stamp = thisImu.header.stamp;
            imuPath.header.frame_id = WORLD_FRAME;
            pubImuPath->publish(imuPath);
        }
    }
}

void imuPreintegration::prepareOdometryMessage( nav_msgs::msg::Odometry &odometry, 
const sensor_msgs::msg::Imu &thisImu, const gtsam::NavState &currentState){
    
    Eigen::Quaterniond q_w_curr;    
    if (config_.use_imu_roll_pitch) {
        q_w_curr = Eigen::Quaterniond(thisImu.orientation.w, thisImu.orientation.x, thisImu.orientation.y, thisImu.orientation.z);
    } else {
        q_w_curr = Eigen::Quaterniond(currentState.quaternion().w(), currentState.quaternion().x(),
                                      currentState.quaternion().y(), currentState.quaternion().z());
    }

    gtsam::Rot3 imuRot(q_w_curr);
    gtsam::Pose3 imuPose = gtsam::Pose3(imuRot, currentState.position());
    gtsam::Pose3 lidarPoseOpt = imuPose.compose(imu2Lidar);

    Eigen::Vector3d velocity_w_curr(currentState.velocity().x(), currentState.velocity().y(),
                                    currentState.velocity().z());
    Eigen::Vector3d velocity_curr = currentState.quaternion().inverse() * velocity_w_curr;
    
    
    odometry.header.stamp = thisImu.header.stamp;
    odometry.header.frame_id = WORLD_FRAME;
    odometry.child_frame_id = SENSOR_FRAME;
    
    Eigen::Quaterniond q_w_lidar(lidarPoseOpt.rotation().toQuaternion().w(), lidarPoseOpt.rotation().toQuaternion().x(),
                                    lidarPoseOpt.rotation().toQuaternion().y(), lidarPoseOpt.rotation().toQuaternion().z());
    q_w_lidar.normalized();

    odometry.pose.pose.position.x = lidarPoseOpt.translation().x();
    odometry.pose.pose.position.y = lidarPoseOpt.translation().y();
    odometry.pose.pose.position.z = lidarPoseOpt.translation().z();
    odometry.pose.pose.orientation.x = q_w_lidar.x();
    odometry.pose.pose.orientation.y = q_w_lidar.y();
    odometry.pose.pose.orientation.z = q_w_lidar.z();
    odometry.pose.pose.orientation.w = q_w_lidar.w();

    odometry.twist.twist.linear.x = velocity_curr.x();
    odometry.twist.twist.linear.y = velocity_curr.y();;
    odometry.twist.twist.linear.z = velocity_curr.z();;
    odometry.twist.twist.angular.x =
            thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry.twist.twist.angular.y =
            thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry.twist.twist.angular.z =
            thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    odometry.pose.covariance[0] = double(RESULT);

    odometry.pose.covariance[1] = prevBiasOdom.accelerometer().x();
    odometry.pose.covariance[2] = prevBiasOdom.accelerometer().y();
    odometry.pose.covariance[3] = prevBiasOdom.accelerometer().z();
    odometry.pose.covariance[4] = prevBiasOdom.gyroscope().x();
    odometry.pose.covariance[5] = prevBiasOdom.gyroscope().y();
    odometry.pose.covariance[6] = prevBiasOdom.gyroscope().z();
    odometry.pose.covariance[7] = config_.imuGravity;


    Eigen::Matrix3d R_wb = currentState.attitude().matrix(); // world<-body
    Eigen::Vector3d f_b(thisImu.linear_acceleration.x,
                    thisImu.linear_acceleration.y,
                    thisImu.linear_acceleration.z);
    // For static: g_w_est = -R_wb * f_b should be ~ [0,0,-g]
    // Eigen::Vector3d g_w_est = -R_wb * f_b;
   // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1,
   //     "g_w_est = [%.3f, %.3f, %.3f], |g|=%.3f", g_w_est.x(), g_w_est.y(), g_w_est.z(), g_w_est.norm());
}


} // end namespace super_odometry