//
// Created by shibo zhao on 2020-09-27.
//
#include "super_odometry/ImuPreintegration/imuPreintegration_release.h"
#include "super_odometry/config/parameter.h"

// For TransformFusion
static double lidarOdomTime2=-1;
deque<nav_msgs::msg::Odometry> imuOdomQueue;
static Eigen::Affine3f lidarOdomAffine;
namespace super_odometry {

    imuPreintegration::imuPreintegration(const rclcpp::NodeOptions & options)
    : Node("imu_preintegration_node", options) {
    }
    
    void imuPreintegration::initInterface() {
        //! Callback Groups - Use MutuallyExclusive for thread safety
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

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
            IMU_TOPIC, 10,
            std::bind(&imuPreintegration::imuHandler, this,
                        std::placeholders::_1), sub_options);
        subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            ProjectName+"/laser_odometry", 5,
            std::bind(&imuPreintegration::laserodometryHandler, this,
                        std::placeholders::_1), sub_options);
        // subVisualOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "vins_estimator/imu_propagate", 10,
        //     std::bind(&imuPreingration::visualodometryHandler, this,
        //                 std::palceholders::_1));

        // pubImuOdometry = this->create_publisher<nav_msgs::msg::Odometry>(
        //     ProjectName+"/integrated_to_init_incremental", 10);
        pubImuOdometry2 = this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/state_estimation", 10);
        pubHealthStatus = this->create_publisher<std_msgs::msg::Bool>(
            ProjectName+"/state_estimation_health", 1);
        pubImuPath = this->create_publisher<nav_msgs::msg::Path>(
            ProjectName+"/imuodom_path", 1);
        // pubImuOdometrySmooth = this->create_publisher<nav_msgs::msg::Odometry>(
        //     ProjectName+"/integrated_to_init2", 2000);

        tfMap2Odom = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfOdom2BaseLink = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // set relevant parameter
        // Add safety check for gravity value
        if (config_.imuGravity <= 0.0 || config_.imuGravity > 20.0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid gravity value: %f", config_.imuGravity);
            return;
        }
        std::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(config_.imuGravity);
        
        // std::shared_ptr<gtsam::PreintegrationParams> p(std::make_shared<gtsam::PreintegrationParams>(config_.imuGravity));
        p->accelerometerCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) *
                                   pow(1e-4, 2); // error committed in integrating position from velocities

        gtsam::imuBias::ConstantBias prior_imu_bias(
                (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVisualPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());         // rad,rad,rad,m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);                      // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6,
                                                             1e-1);                    // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, config_.lidar_correction_noise); // meter
       // correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m

        noMotionNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished()); // rad,rad,rad,m, m, m
        noVelocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);

        noiseModelBetweenBias = (gtsam::Vector(6)
                << config_.imuAccBiasN,
                config_.imuAccBiasN, config_.imuAccBiasN, config_.imuGyrBiasN, config_.imuGyrBiasN, config_.imuGyrBiasN)
                .finished();
        imuIntegratorImu_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias); // setting up the IMU integration for IMU message
        imuIntegratorOpt_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias); // setting up the IMU integration for optimization

        //set extrinsic matrix for laser and imu
        if (PROVIDE_IMU_LASER_EXTRINSIC) {
            imu2Lidar = gtsam::Pose3(gtsam::Rot3(imu_laser_R), gtsam::Point3(imu_laser_T));
            lidar2Imu = imu2Lidar.inverse();
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
        this->declare_parameter<float>("imu_preintegration_node.g_norm", 9.8);
        this->declare_parameter<float>("imu_preintegration_node.lidar_correction_noise",0.01);
        this->declare_parameter<float>("imu_preintegration_node.smooth_factor",0.9);
        this->declare_parameter<bool>("imu_preintegration_node.use_imu_roll_pitch",true);
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
        config_.use_imu_roll_pitch = this->get_parameter("imu_preintegration_node.use_imu_roll_pitch").as_bool();
        config_.imu_acc_x_limit = this->get_parameter("imu_preintegration_node.imu_acc_x_limit").as_double();
        config_.imu_acc_y_limit = this->get_parameter("imu_preintegration_node.imu_acc_y_limit").as_double();
        config_.imu_acc_z_limit = this->get_parameter("imu_preintegration_node.imu_acc_z_limit").as_double();
        
        // Validate acceleration limits
        if (config_.imu_acc_x_limit <= 0 || config_.imu_acc_y_limit <= 0 || config_.imu_acc_z_limit <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid IMU acceleration limits: x=%f, y=%f, z=%f", 
                        config_.imu_acc_x_limit, config_.imu_acc_y_limit, config_.imu_acc_z_limit);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "config_.use_imu_roll_pitch: %d", config_.use_imu_roll_pitch);

        RCLCPP_INFO(this->get_logger(), "imuAccNoise: %f  imuAccBiasN: %f imuGyrNoise %f imuGyrBiasN %f imuGravity %f",
                    config_.imuAccNoise, config_.imuAccBiasN, config_.imuGyrNoise, config_.imuGyrBiasN, config_.imuGravity);

        std::string sensorType;
        // Use the global SENSOR variable instead of trying to get the parameter directly
        sensorType = SENSOR;
        if (sensorType.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "no sensor parameters");
            return false;
        }
         if (sensorType == "velodyne")
        {
            config_.sensor = SensorType::VELODYNE;
            std::cerr << "sensor = VELODYNE" << std::endl;
        }
        else if (sensorType == "ouster")
        {
            config_.sensor = SensorType::OUSTER;
            std::cerr << "sensor=OUSTER" << std::endl;
        }
        else if (sensorType == "livox")
        {
            config_.sensor = SensorType::LIVOX;
            std::cerr << "sensor=LIVOX" << std::endl;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                    "Invalid sensor type: %s. (must be either 'velodyne' or 'ouster')", sensorType.c_str());
            rclcpp::shutdown();
        }

        return true;

    }



    Eigen::Affine3f imuPreintegration::odom2affine(nav_msgs::msg::Odometry odom) {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf2::Quaternion orientation;
        tf2::fromMsg(odom.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void imuPreintegration::resetOptimization() {
        try {
            gtsam::ISAM2Params optParameters;
            optParameters.relinearizeThreshold = 0.1;
            optParameters.relinearizeSkip = 1;
            optimizer = gtsam::ISAM2(optParameters);

            gtsam::NonlinearFactorGraph newGraphFactors;
            graphFactors = newGraphFactors;

            gtsam::Values NewGraphValues;
            graphValues = NewGraphValues;
            
            RCLCPP_DEBUG(this->get_logger(), "ISAM2 optimizer reset successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to reset ISAM2 optimizer: %s", e.what());
        }
    }

    void imuPreintegration::resetParams() {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void imuPreintegration::addNoMotionFactor(const FrameId &from_id, const FrameId &to_id)
    {

        Eigen::Matrix3d relative_motion;
        Eigen::Vector3d relative_trans;

        graphFactors.push_back(
                std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                        gtsam::Symbol('x', from_id),
                        gtsam::Symbol('x', to_id),
                        gtsam::Pose3(gtsam::Rot3(relative_motion.setIdentity()),
                                     gtsam::Point3(relative_trans.setZero())),
                        noMotionNoise));

        // debug_info_.numAddedNoMotionF_++;
        // if (VLOG_IS_ON(10)) {
        //     std::cout << "No motion detected, adding no relative motion prior"
        //               << std::endl;
        // }
    }

    void imuPreintegration::addZeroVelocityPrior(const FrameId &frame_id) {
        // VLOG(10) << "No motion detected, adding zero velocity prior.";
        graphFactors.push_back(
                std::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
                        gtsam::Symbol('v', frame_id),
                        gtsam::Vector3::Zero(),
                        noVelocityNoise));
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

        // 0. initialize system
        resetOptimization();
        //TODO: it seems that we don't need this  process if we use ringbuffer


        while (!imuQueOpt.empty())
        {
            if (secs(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
            {
                lastImuT_opt = secs(&imuQueOpt.front());
                imuQueOpt.pop_front();
            }
            else
                break;
        }


        // initial pose
        prevPose_ = lidarPose.compose(lidar2Imu);

        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                   priorPoseNoise);
        graphFactors.add(priorPose);
        // initial velocity
        prevVel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_,
                                                    priorVelNoise);
        graphFactors.add(priorVel);
        // initial bias
        prevBias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
                B(0), prevBias_, priorBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        // addZeroVelocityPrior(0);
        //     addNoMotionFactor(key - 1, key);

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
        auto diff  = curPose.translation() - propState_.pose().translation();
        // RCLCPP_WARN(this->get_logger(), "diff norm: %f", diff.norm());

        // if (diff.norm() > 0.2)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Large difference between imu and lidar pose estimation");
        //     // resetParams();
        //     // return;
        //     correctionNoise =  gtsam::noiseModel::Isotropic::Sigma(6, lidar_correction_noise*100); // meter
        // }else{
        //     correctionNoise =  gtsam::noiseModel::Isotropic::Sigma(6, lidar_correction_noise); // meter

        // }
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose,
                                                     correctionNoise);
        graphFactors.add(pose_factor);
                // add imu factor to graph
        
        const gtsam::PreintegratedImuMeasurements &preint_imu =
                dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(
                        *imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key),
                                    B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                gtsam::noiseModel::Diagonal::Sigmas(
                        sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        
  
        // optimize
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

        graphFactors.resize(0);
        graphValues.clear();

        if (systemSolvedSuccessfully) {

            // RCLCPP_INFO_STREAM(this->get_logger(), "key: " << key);
            // RCLCPP_INFO_STREAM(this->get_logger(), "X shape" <<  X);
            // Overwrite the beginning of the preintegration for the next step.
            gtsam::Values result = optimizer.calculateEstimate();
            prevPose_ = result.at<gtsam::Pose3>(X(key));
            prevVel_ = result.at<gtsam::Vector3>(V(key));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
            // Reset the optimization preintegration object.
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);


        }        
        return systemSolvedSuccessfully;
    }

    void imuPreintegration::repropagate_imuodometry(double currentCorrectionTime) {
        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;

        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && secs(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = secs(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::msg::Imu *thisImu = &imuQueImu[i];
                double imuTime = secs(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 200.0) :(imuTime - lastImuQT);
                lastImuQT = imuTime;

                if(dt < 0.001 || dt > 0.5) 
                    dt = 0.005;

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

    }

    void imuPreintegration::process_imu_odometry(double currentCorrectionTime, gtsam::Pose3 relativePose) {

        // reset graph for speed
        if (key > 100) {
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

    //    RCLCPP_INFO_STREAM(this->get_logger(), "key: " << key);
        // 4. reprogate_imuodometry
        repropagate_imuodometry(currentCorrectionTime);
        ++key;

        doneFirstOpt = true;
    }

    bool imuPreintegration::failureDetection(const gtsam::Vector3 &velCur,
                                             const gtsam::imuBias::ConstantBias &biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30) {
            RCLCPP_WARN(this->get_logger(), "Large velocity, reset IMU-preintegration!");
            return true;
        }



        // if (use_no_motion_prior && vel.norm() < 0.03) {
        //     addZeroVelocityPrior(key);
        //     addNoMotionFactor(key - 1, key);
        //     RCLCPP_INFO(this->get_logger(), " Detect small velocity and Reset velocity to zero ! ");
        // }


        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(),
                           biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(),
                           biasCur.gyroscope().z());

        // RCLCPP_WARN(this->get_logger(), " ba.norm: %f | bg.norm: %f", ba.norm(), bg.norm());
        if (ba.norm() > 2.0 || bg.norm() > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

# if 0
    void imuPreintegration::synchronize_measurements(std::deque<nav_msgs::msg::Odometry::SharedPtr> &laserodomQue,
                                                     std::deque<nav_msgs::msg::Odometry::SharedPtr> &visualodomQue,
                                                     std::deque<sensor_msgs::msg::Imu> &imuQue)
    {

        use_visualodom = false;
        use_laserodom = false;
        use_onlyimu = false;
        health_status = false;

        if (imuQue.empty())
            return;

        if (laserodomQue.empty() && visualodomQue.empty())
        {
            use_onlyimu = true;
            health_status = false;
            return;
        }

        if (!laserodomQue.empty() && !visualodomQue.empty() && !imuQue.empty())
        {
            mBuf.lock();
            health_status = true;
            while (!visualodomQue.empty() &&
                   secs(visualodomQue.front()) < secs(laserodomQue.front()) - 0.05)
                visualodomQue.pop_front();

            if (visualodomQue.empty())
            {
                mBuf.unlock();
                use_visualodom = false;
            }
            else
            {
                use_visualodom = true;
            }

            while (!laserodomQue.empty() && secs(visualodomQue.front()) 
                                            > secs(laserodomQue.front()) + 0.05)
            {
                laserodomQue.pop_front();
            }

            if (laserodomQue.empty())
            {
                mBuf.unlock();
                use_laserodom = false;
            }
            else
            {
                use_laserodom = true;
            }

            if (!laserodomQue.empty() && !visualodomQue.empty())
            {
                double timelaserodom = secs(&laserodomQue.front());
                double timevisualodom = secs(&visualodomQue.front());

                if (abs(timelaserodom - timevisualodom) > 0.05)
                {
                    mBuf.unlock();
                    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> unsync laserodom and visual odom.\033[0m");
                }
            }
            mBuf.unlock();
        }
        else if (!laserodomQue.empty() && !imuQue.empty())
        {
            use_laserodom = true;
            health_status = true;
        }
        else if (!visualodomQue.empty() && !imuQue.empty())
        {
            use_visualodom = true;
            health_status = false;
        }

        // RCLCPP_WARN(this->get_logger(), "health_status %d", health_status);
    }

#endif

    void imuPreintegration::laserodometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        // RCLCPP_INFO(this->get_logger(), "laserOdometryCB from /aft_mapped_to_init_incremental");
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
        // bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
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


#if 1
        // 2. safe lannding process
        double latest_imu_time;

        latest_imu_time = secs(&imuQueImu.back());


      //    LOG(INFO) << "abs(latest_laser_time - latest_imu_time (including process)): " << abs(lidarOdomTime - latest_imu_time);

        if (lidarOdomTime - latest_imu_time < imu_laser_timedelay) {
            RESULT = IMU_STATE::SUCCESS;
            health_status = true;
          
          if((int)odomMsg->pose.covariance[0] == 1)
           {
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

                // odometry.header.stamp = odomMsg->header.stamp;
                // odometry.header.frame_id = WORLD_FRAME;
                // odometry.child_frame_id = SENSOR_FRAME;

                // odometry.pose.pose.position.x = odomMsg->pose.pose.position.x;
                // odometry.pose.pose.position.y = odomMsg->pose.pose.position.y;
                // odometry.pose.pose.position.z = odomMsg->pose.pose.position.z;
                // odometry.pose.pose.orientation.x = odomMsg->pose.pose.orientation.x;
                // odometry.pose.pose.orientation.y = odomMsg->pose.pose.orientation.y;
                // odometry.pose.pose.orientation.z = odomMsg->pose.pose.orientation.z;
                // odometry.pose.pose.orientation.w = odomMsg->pose.pose.orientation.w;

                // odometry.twist.twist.linear.x = velocity_curr.x();
                // odometry.twist.twist.linear.y = velocity_curr.y();
                // odometry.twist.twist.linear.z = velocity_curr.z();
                // odometry.pose.covariance[0] = double(RESULT);
                // pubImuOdometrySmooth->publish(odometry);

                std_msgs::msg::Bool health_status_msg;
                health_status_msg.data = health_status;
                pubHealthStatus->publish(health_status_msg);
            }
        }

        last_frame = cur_frame;
#endif
        //   receive_first_laserodom=true;
        last_processed_lidar_time = lidarOdomTime;
    }

    void imuPreintegration::visualodometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        RCLCPP_INFO(this->get_logger(), "visualodometryHandler from");
        std::lock_guard<std::mutex> lock(mBuf);
        if (imuQueOpt.empty())
            return;

        visualOdomBuf.addMeas(odomMsg, secs(odomMsg));

    }



    //TODO: need to consider the extrinsic matrix of imu and lidar
    sensor_msgs::msg::Imu imuPreintegration::imuConverter(const sensor_msgs::msg::Imu &imu_in) {
        sensor_msgs::msg::Imu imu_out = imu_in;
        
        Eigen::Matrix3d imu_laser_R_Gravity;
        imu_laser_R_Gravity=imu_Init->imu_laser_R_Gravity;

        Eigen::Vector3d rpy;
        rpy=imu_Init->rotationMatrixToRPY(imu_laser_R_Gravity);
 

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

        acc=imu_laser_R_Gravity*acc;
        acc = acc + ((gyr - gyr_pre) * 200).cross(- imu_laser_T) + gyr.cross(gyr.cross(-imu_laser_T));
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

    void imuPreintegration::imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw) 
    {
     
        std::lock_guard<std::mutex> lock(mBuf);
        
        sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);
        
        // Removed problematic assert that could cause crashes
        // assert(imu_raw->linear_acceleration.x!=thisImu.linear_acceleration.x); 


        if(config_.sensor==SensorType::LIVOX and imu_init_success == false)
        {
          Imu::Ptr imudata = std::make_shared<Imu>();
          imudata->time = imu_raw->header.stamp.sec + imu_raw->header.stamp.nanosec * 1e-9;
          imudata->acc = Eigen::Vector3d(imu_raw->linear_acceleration.x,imu_raw->linear_acceleration.y,imu_raw->linear_acceleration.z);
          imudata->gyr = Eigen::Vector3d(imu_raw->angular_velocity.x,imu_raw->angular_velocity.y,imu_raw->angular_velocity.z);
          imudata->q_w_i=Eigen::Quaterniond(imu_raw->orientation.w, 
          imu_raw->orientation.x, imu_raw->orientation.y, imu_raw->orientation.z);

          // Add safety check before adding measurement
          if (imuBuf.getSize() < imuBuf.size) {
              imuBuf.addMeas(imudata, imudata->time);
          } else {
              RCLCPP_WARN(this->get_logger(), "IMU buffer full, skipping measurement");
              return;
          }

          double first_imu_time = 0.0; 
          if (imuBuf.getFirstTime(first_imu_time)) {
              if (imudata->time - first_imu_time > 0.1 and imu_init_success == false) {   
                  //TODO: IMUInit might be not necessary since it is only for accleration 
                  try {
                      imu_Init->imuInit(imuBuf);
                      imu_init_success=true;
                      imuBuf.clean(imudata->time);
                      RCLCPP_INFO(this->get_logger(), "IMU Initialization Process Finish!");
                  } catch (const std::exception& e) {
                      RCLCPP_ERROR(this->get_logger(), "IMU initialization failed: %s", e.what());
                  }
              }
          }
        }

        if(config_.sensor==SensorType::LIVOX)
        {
           double gravity = 9.8105;
           Eigen::Vector3d acc;
           acc=Eigen::Vector3d(thisImu.linear_acceleration.x,thisImu.linear_acceleration.y,thisImu.linear_acceleration.z);
           acc= acc*gravity/imu_Init->acc_mean.norm();
           thisImu.linear_acceleration.x=acc.x();
           thisImu.linear_acceleration.y=acc.y();
           thisImu.linear_acceleration.z=acc.z();
        }
        
        if(imu_init_success==false) 
        {
            return;
        }
        

        //Add the IMU Initialization for LIVOX LiDAR
        double imuTime = secs(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 200.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;
        
        if(dt < 0.001 || dt > 0.5) 
            dt = 0.005;
            // return;

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        double roll, pitch, yaw;
        tf2::Quaternion orientation;
        tf2::fromMsg(thisImu.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        // RCLCPP_INFO(this->get_logger(), "Aroll, pitch, yaw %f, %f, %f", roll, pitch, yaw);

        tf2::Quaternion yaw_quat;
        yaw_quat.setRPY(0,0,-yaw);

        orientation = yaw_quat*orientation;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        // ROS_INFO_THROTTLE(this->get_logger(), 1.0, "roll, pitch, yaw %f, %f, %f", roll, pitch, yaw);


        if (doneFirstOpt == false)
            return;



        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(
                gtsam::Vector3(thisImu.linear_acceleration.x,
                               thisImu.linear_acceleration.y,
                               thisImu.linear_acceleration.z),
                gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y,
                               thisImu.angular_velocity.z),
                dt);

        // predict odometry
        gtsam::NavState currentState =
                imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        tf2::Quaternion state_q(currentState.quaternion().x(), currentState.quaternion().y(), currentState.quaternion().z(), currentState.quaternion().w());

        tf2::Matrix3x3(state_q).getRPY(roll, pitch, yaw);
        // tf2::Quaternion yaw_quat;
        yaw_quat.setRPY(0,0,yaw);
        orientation = yaw_quat*orientation;

        // publish odometry
        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = WORLD_FRAME;
        odometry.child_frame_id = SENSOR_FRAME;
        
        // TODO: convert current Velocity State from world frame to current frame
        Eigen::Quaterniond q_w_curr;
        if (config_.use_imu_roll_pitch)
        {
            // RCLCPP_WARN(this->get_logger(), "imuPreintegration: use imu roll pitch");
            q_w_curr = Eigen::Quaterniond(orientation.w(), orientation.x(), orientation.y(), orientation.z());
            // q_w_curr = Eigen::Quaterniond(currentState.quaternion().w(), currentState.quaternion().x(),
            //                               currentState.quaternion().y(), currentState.quaternion().z());
        }
        else
        {
            q_w_curr = Eigen::Quaterniond(currentState.quaternion().w(), currentState.quaternion().x(),
                                          currentState.quaternion().y(), currentState.quaternion().z());
        }
        // Eigen::Quaterniond q_w_curr(currentState.quaternion().w(), currentState.quaternion().x(),
        //                             currentState.quaternion().y(), currentState.quaternion().z());
        gtsam::Rot3 imuRot(q_w_curr);
            // transform imu pose to ldiar
        gtsam::Pose3 imuPose =
                gtsam::Pose3(imuRot, currentState.position());
        gtsam::Pose3 lidarPoseOpt = imuPose.compose(imu2Lidar);

        Eigen::Vector3d velocity_w_curr(currentState.velocity().x(), currentState.velocity().y(),
                                        currentState.velocity().z());

        //     Eigen::Vector3d velocity_w_curr(0.05*currentState.velocity().x()+0.95*prevStateOdom.velocity().x(),
        //                                        0.05*currentState.velocity().y()+0.95*prevStateOdom.velocity().y(),
        //                                        0.1*currentState.velocity().z()+0.9*prevStateOdom.velocity().z());

        Eigen::Vector3d velocity_curr = currentState.quaternion().inverse() * velocity_w_curr;



        // odometry.pose.pose.position.x = currentState.position().x();
        // odometry.pose.pose.position.y = currentState.position().y();
        // odometry.pose.pose.position.z = currentState.position().z();

        // odometry.pose.pose.orientation.x = currentState.quaternion().x();
        // odometry.pose.pose.orientation.y = currentState.quaternion().y();
        // odometry.pose.pose.orientation.z = currentState.quaternion().z();
        // odometry.pose.pose.orientation.w = currentState.quaternion().w();

        // odometry.pose.pose.orientation.x = q_w_curr.x();
        // odometry.pose.pose.orientation.y = q_w_curr.y();
        // odometry.pose.pose.orientation.z = q_w_curr.z();
        // odometry.pose.pose.orientation.w = q_w_curr.w();

        // odometry.twist.twist.linear.x = velocity_curr.x();
        // odometry.twist.twist.linear.y = velocity_curr.y();
        // odometry.twist.twist.linear.z = velocity_curr.z();
        // odometry.twist.twist.angular.x =
        //         thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        // odometry.twist.twist.angular.y =
        //         thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        // odometry.twist.twist.angular.z =
        //         thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        // odometry.pose.covariance[0] = double(RESULT);
        // pubImuOdometry->publish(odometry);


        nav_msgs::msg::Odometry odometry2;
        odometry2.header.stamp = thisImu.header.stamp;
        odometry2.header.frame_id = WORLD_FRAME;
        odometry2.child_frame_id = SENSOR_FRAME;
        
        Eigen::Quaterniond q_w_lidar(lidarPoseOpt.rotation().toQuaternion().w(), lidarPoseOpt.rotation().toQuaternion().x(),
                                     lidarPoseOpt.rotation().toQuaternion().y(), lidarPoseOpt.rotation().toQuaternion().z());
        q_w_lidar.normalized();

        odometry2.pose.pose.position.x = lidarPoseOpt.translation().x();
        odometry2.pose.pose.position.y = lidarPoseOpt.translation().y();
        odometry2.pose.pose.position.z = lidarPoseOpt.translation().z();
        odometry2.pose.pose.orientation.x = q_w_lidar.x();
        odometry2.pose.pose.orientation.y = q_w_lidar.y();
        odometry2.pose.pose.orientation.z = q_w_lidar.z();
        odometry2.pose.pose.orientation.w = q_w_lidar.w();

        odometry2.twist.twist.linear.x = velocity_curr.x();
        odometry2.twist.twist.linear.y = velocity_curr.y();;
        odometry2.twist.twist.linear.z = velocity_curr.z();;
        odometry2.twist.twist.angular.x =
                thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry2.twist.twist.angular.y =
                thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry2.twist.twist.angular.z =
                thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        odometry2.pose.covariance[0] = double(RESULT);

        odometry2.pose.covariance[1] = prevBiasOdom.accelerometer().x();
        odometry2.pose.covariance[2] = prevBiasOdom.accelerometer().y();
        odometry2.pose.covariance[3] = prevBiasOdom.accelerometer().z();
        odometry2.pose.covariance[4] = prevBiasOdom.gyroscope().x();
        odometry2.pose.covariance[5] = prevBiasOdom.gyroscope().y();
        odometry2.pose.covariance[6] = prevBiasOdom.gyroscope().z();
        odometry2.pose.covariance[7] = config_.imuGravity;
        
        frame_count++;
        if(frame_count%4==0)
            pubImuOdometry2->publish(odometry2);

        std_msgs::msg::Bool health_status_msg;
        health_status_msg.data = health_status;
        pubHealthStatus->publish(health_status_msg);
       
        tf2_ros::TransformBroadcaster br(this);
        // tf2::Transform transform;
        geometry_msgs::msg::TransformStamped transform_stamped_;
        tf2::Transform transform;
        transform_stamped_.header.stamp  = thisImu.header.stamp;
        transform_stamped_.header.frame_id = WORLD_FRAME;
        transform_stamped_.child_frame_id = SENSOR_FRAME;
        
        tf2::Quaternion q;
        transform.setOrigin(tf2::Vector3(odometry2.pose.pose.position.x, 
        odometry2.pose.pose.position.y, odometry2.pose.pose.position.z));

        q.setW(q_w_lidar.w());
        q.setX(q_w_lidar.x());
        q.setY(q_w_lidar.y());
        q.setZ(q_w_lidar.z());
        transform.setRotation(q);
        transform_stamped_.transform = tf2::toMsg(transform);
        if(frame_count%4==0)
            br.sendTransform(transform_stamped_);

        // publish IMU path
        static nav_msgs::msg::Path imuPath;
        static double last_path_time = -1;
        double curimuTime = secs(&thisImu);
        if (curimuTime - last_path_time > 0.1)
        {
            last_path_time = curimuTime;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = thisImu.header.stamp;
            pose_stamped.header.frame_id = WORLD_FRAME;
            pose_stamped.pose = odometry2.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while (!imuPath.poses.empty() &&
                   abs(secs(&imuPath.poses.front()) -
                       secs(&imuPath.poses.back())) > 3.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath->get_subscription_count() != 0)
            {
                imuPath.header.stamp = thisImu.header.stamp;
                imuPath.header.frame_id = WORLD_FRAME;
                pubImuPath->publish(imuPath);
            }
        }

        use_laserodom = false;
        use_visualodom = false;

    }

} // end namespace superodom