//
// Created by ubuntu on 2020/9/26.
//

#pragma once
#ifndef LIDARSLAM_H
#define LIDARSLAM_H

#include <atomic>

#include <tbb/concurrent_vector.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "super_odometry/sensor_data/pointcloud/LidarPoint.h"
#include "super_odometry/LidarProcess/LocalMap.h"


#include "super_odometry/utils/EigenTypes.h"
#include "super_odometry/utils/Twist.h"
#include <super_odometry_msgs/msg/optimization_stats.hpp>

#include <sophus/se3.hpp>
#include <std_msgs/msg/float32.hpp>
#include <filesystem>

#include "super_odometry/LidarProcess/factor/SE3AbsolutatePoseFactor.h"
#include "super_odometry/LidarProcess/factor/lidarOptimization.h"
#include "super_odometry/LidarProcess/factor/pose_local_parameterization.h"
#include <ceres/ceres.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <Eigen/Dense>
#include <glog/logging.h>
#include "super_odometry/tic_toc.h"
#include "super_odometry/config/parameter.h"
#include "super_odometry/utils/superodom_utils.h"

namespace super_odometry {

    class LidarSLAM {


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Point = pcl::PointXYZI;
        using PointCloud = pcl::PointCloud<Point>;
        enum class PredictionSource {
            IMU_ORIENTATION, LIO_ODOM, VIO_ODOM, NEURAL_IMU_ODOM, CONSTANT_VELOCITY
        };


    public:
        
        struct RotatedAxes {
        Eigen::Vector3f x, y, z;};


        enum class MatchingMode {
            EGO_MOTION = 0, LOCALIZATION = 1
        };

        enum UndistortionMode {
            //! No undistortion is performed:
            //! -End scan pose is optimized using rigid registration of raw scan and map.
            //! -Raw input scan is added to maps.
            NONE = 0,

            //! Minimal undistortion is performed:
            //! - begin scan pose is linearly interpolated between previous and current end scan poses.
            //! - End scan pose is optimized using rigid registration of undistorted scan and map.
            //! - Scan is linearly undistorted between begin and end scan poses.
            APPROXIMATED = 1,

            //! Ceres-optimized undistorted is performed:
            //! -both begin and end scan are optimized using registration of undistorted scan
            //! and map.
            //! -Scan is linearly undistorted between begin and scan poses.
            OPTIMIZED = 2
        };

        //! Result of keypoint matching, explaining rejection
        enum MatchingResult : uint8_t {
            SUCCESS = 0,               // keypoint has been successfully matched
            NOT_ENOUGH_NEIGHBORS = 1,  // not enough neighbors to match keypoint
            NEIGHBORS_TOO_FAR = 2,     // neighbors are too far to match keypoint
            BAD_PCA_STRUCTURE = 3,     // PCA eigenvalues analysis discards neighborhood fit to model
            INVAVLID_NUMERICAL = 4,    // optimization parameter computation has numerical invalidity
            MSE_TOO_LARGE = 5,         // mean squared error to model is too large to accept fitted model
            UNKNON = 6,                // unkown status (matching not performed yet)
            nRejectionCauses = 7
        };

        enum Feature_observability : uint8_t {
            rx_cross = 0,               // evaluate for x rotation estimation
            neg_rx_cross = 1,           // evaluate for neg x  rotation estimation
            ry_cross = 2,               // evaluate for y rotation estimation
            neg_ry_cross = 3,           // evaluate for neg y rotation estimation
            rz_cross = 4,               // evaluate for z rotation estimation
            neg_rz_cross = 5,           // evaluate for neg z rotation estimation
            tx_dot = 6,                 // evaluate for x translation
            ty_dot = 7,                 // evaluate for y translation
            tz_dot = 8,                    // evaluate for z translation
            nFeatureObs = 9
        };

        enum FeatureType : uint8_t {
            EdgeFeature = 0,
            PlaneFeature = 1,
        };


    public:

        struct LaserOptSet {
            tf2::Quaternion imu_roll_pitch;
            bool  debug_view_enabled;
            bool  use_imu_roll_pitch;
            float velocity_failure_threshold;
            float yaw_ratio;
            int max_surface_features;
        };

        //! Estimation of registration error
        struct RegistrationError {
            // Estimation of the maximum position error
            double PositionError = 0.;
            double PositionUncertainty = 0.;
            double MaxPositionError = 0.1;
            double PosInverseConditionNum = 1.0;

            //Estimation of Lidar Uncertainty
            Eigen::VectorXf LidarUncertainty;

            // Direction of the maximum position error
            Eigen::Vector3d PositionErrorDirection = Eigen::Vector3d::Zero();

            // Estimation of the maximum orientation error (in radians)
            double OrientationError = 0.;
            double OrientationUncertainty = 0.;
            double MaxOrientationError = 10;
            double OriInverseConditionNum = 1.0;
            // Direction of the maximum orientation error
            Eigen::Vector3d OrientationErrorDirection = Eigen::Vector3d::Zero();

            // Covariance matrix encoding the estimation of the pose's errors about the 6-DoF parameters
            // (DoF order :  X, Y, Z,rX, rY, rZ)
            Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Covariance = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero();
        };

        struct eigenValue // Eigen Value ,lamada1 > lamada2 > lamada3;
        {
            double lamada1;
            double lamada2;
            double lamada3;
        };

        struct eigenVector //the eigen vector corresponding to the eigen value
        {
            Eigen::Vector3f principalDirection;
            Eigen::Vector3f normalDirection;
        };

        struct pcaFeature //PCA
        {
            eigenValue values;
            eigenVector vectors;
            double curvature;
            double linear;
            double planar;
            double spherical;
            double linear_2;
            double planar_2;
            double spherical_2;
            pcl::PointNormal pt;
            size_t ptId;
            size_t ptNum = 0;
            std::vector<int> neighbor_indices;
            std::array<int, 4> observability;
            double rx_cross;
            double neg_rx_cross;
            double ry_cross;
            double neg_ry_cross;
            double rz_cross;
            double neg_rz_cross;
            double tx_dot;
            double ty_dot;
            double tz_dot;
        };

        struct LidarOdomUncertainty {
            double uncertainty_x;
            double uncertainty_y;
            double uncertainty_z;
            double uncertainty_roll;
            double uncertainty_pitch;
            double uncertainty_yaw;
        };

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyX;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyY;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyZ;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyRoll;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyPitch;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubUncertaintyYaw;;

        struct OptimizationParameter {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            MatchingResult match_result;
            FeatureType feature_type;
            pcaFeature feature;
            Eigen::Matrix3d Avalue;
            Eigen::Vector3d Pvalue;
            Eigen::Vector3d Xvalue;
            Eigen::Vector3d NormDir;
            double negative_OA_dot_norm;
            std::pair<Eigen::Vector3d, Eigen::Vector3d> corres;
            double residualCoefficient;
            double TimeValue;
        };

    public:
        LocalMap localMap;
    
        super_odometry_msgs::msg::OptimizationStats stats;
        RegistrationError LocalizationUncertainty;
        LidarOdomUncertainty lidarOdomUncer;
        LaserOptSet OptSet;

        Transformd T_w_lidar;
        Transformd last_T_w_lidar;
        Transformd T_w_initial_guess;
        Eigen::Vector3i pos_in_localmap;

        int frame_count;
        int laser_imu_sync;
        int startupCount = 0;

        float Pos_degeneracy_threshold;
        float Ori_degeneracy_threshold;
        float Visual_confidence_factor;
        
        std::string map_dir;
        float init_x;
        float init_y;
        float init_z;
        float init_roll;
        float init_pitch;
        float init_yaw;
        float localization_mode;
        float update_map;
        double lasttimeLaserOdometry;

        PointCloud::Ptr EdgesPoints;
        PointCloud::Ptr PlanarsPoints;


        PointCloud::Ptr WorldEdgesPoints;
        PointCloud::Ptr WorldPlanarsPoints;

        bool bInitialization = false;
        bool isDegenerate = false;
        bool save_ply = false;

        UndistortionMode Undistortion = UndistortionMode::NONE;
        std::array<std::atomic_int, Feature_observability::nFeatureObs> PlaneFeatureHistogramObs;
        std::array<std::atomic_int, MatchingResult::nRejectionCauses> MatchRejectionHistogramLine;
        std::array<std::atomic_int, MatchingResult::nRejectionCauses> MatchRejectionHistogramPlane;
        tbb::concurrent_vector<OptimizationParameter> OptimizationData;

    
        size_t LocalizationICPMaxIter = 4;

        size_t LocalizationLineDistanceNbrNeighbors = 10;
        size_t LocalizationMinmumLineNeighborRejection = 4;
        size_t LocalizationPlaneDistanceNbrNeighbors = 5;

        double LocalizationLineDistancefactor = 5.0;

        double LocalizationLineMaxDistInlier = 0.2;
 
        rclcpp::Node::SharedPtr node_;

        // PLY file saving variables
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_to_save;

    public:
        LidarSLAM();

        void Reset(bool resetLog = true);

        void Localization(bool initialization, PredictionSource predictodom, Transformd T_w_lidar,
                          pcl::PointCloud<Point>::Ptr edge_point, pcl::PointCloud<Point>::Ptr planner_point, double timeLaserOdometry);

        void ComputePointInitAndFinalPose(MatchingMode matchingMode,const Point &p,Eigen::Vector3d &pInit,Eigen::Vector3d &pFinal);

        OptimizationParameter ComputeLineDistanceParameters(LocalMap &local_map, const Point &p);

        OptimizationParameter ComputePlaneDistanceParameters(LocalMap &local_map, const Point &p);

        RegistrationError EstimateRegistrationError(ceres::Problem &problem, const double eigen_thresh);

        void FeatureObservabilityAnalysis(pcaFeature &feature, const Eigen::Vector3d &pFinal, const Eigen::Vector3d &eigenvalues, 
                                          const Eigen::Vector3d &normal_direction, const Eigen::Vector3d &principal_direction);

        inline void ResetDistanceParameters();

        void MannualYawCorrection();

        void initROSInterface(rclcpp::Node::SharedPtr);

        void initializeState(bool initialization, const Transformd&position);

        void processInputClouds(const pcl::PointCloud<Point>::Ptr&edge_point, const pcl::PointCloud<Point>::Ptr&planner_point);

        void initializeMapping(double timeLaserOdometry);

        void performLocalizationAndMapping(PredictionSource predictodom, double timeLaserOdometry);

        void transformAndAddToMap(const pcl::PointCloud<Point>::Ptr&source_cloud, 
        pcl::PointCloud<Point>::Ptr&world_cloud, bool is_edge);

        void updateFeatureStats(size_t edge_count, size_t planner_count);

        bool hasEnoughFeatures();

        void processEdgeFeatures(tbb::concurrent_vector<OptimizationParameter>&features_corres, int &edge_num);

        void processPlannerFeatures(tbb::concurrent_vector<OptimizationParameter>&features_corres, int &planner_num);

        double calculateSamplingRate(size_t num_points);

        bool shouldProcessPoint(size_t index, double sampling_rate);

        void extractFeaturesConstraints(tbb::concurrent_vector<OptimizationParameter>&feature_corres, int &edge_num, int &planner_num);

        void prepareOptimizationState();

        ceres::Problem setupOptimizationProblem(const tbb::concurrent_vector<OptimizationParameter>&features_corres, 
        PredictionSource predictsource, const Transformd&position);

        void addFeatureConstraints(ceres::Problem&problem, const tbb::concurrent_vector<OptimizationParameter>&features_corres);

        bool shouldAddAbsolutePoseConstraints(PredictionSource predictodom);    

        void addAbsolutePoseConstraints(ceres::Problem&problem, const Transformd&position, int good_feature_num);
        
        ceres::Solver::Summary solveOptimizationProblem(ceres::Problem&problem);

        void recordIterationStats(super_odometry_msgs::msg::IterationStats&iter_stats,
        int surf_num, int edge_num, Transformd&previous_T, Transformd&current_T);

        void performPostOptimizationProcessing(double timeLaserOdometry, TicToc &t_opt, super_odometry_msgs::msg::OptimizationStats &stats);

        bool checkMotionThresholds(double timeLaserOdometry, super_odometry_msgs::msg::OptimizationStats &stats);

        void updateOptimizationStats(TicToc &t_opt, super_odometry_msgs::msg::OptimizationStats &stats);
        
        bool initializeAndTransformPoint(const Point &p, Eigen::Vector3d &pInit, Eigen::Vector3d &pFinal);
        
        bool findNearestNeighbors(LocalMap &local_map,
                                    const Eigen::Vector3d &pFinal,
                                    std::vector<Point> &nearest_pts,
                                    std::vector<float> &nearest_dist,
                                    size_t requiredNearest,
                                    size_t min_neighbors,
                                    double square_max_dist,
                                    OptimizationParameter &result);

        bool computePCAForFeature(const std::vector<Point> &nearest_pts,
                                  Eigen::Vector3d &mean,
                                  Eigen::Vector3d &eigenvalues,
                                  Eigen::Matrix3d &eigenvectors,
                                  OptimizationParameter &result,
                                  FeatureType feature_type);

        bool computeFeatureObservability(pcaFeature &feature, const Eigen::Vector3d &pFinal, 
                                          const Eigen::Vector3d &eigenvalues, 
                                          const Eigen::Vector3d &normal_direction, 
                                          const Eigen::Vector3d &principal_direction);

        void setPlaneResults(OptimizationParameter &result, const Eigen::Vector3d &mean, 
                               const Eigen::Vector3d &pInit, const Eigen::Vector3d &plane_normal, 
                               double negative_OA_dot_norm, const pcaFeature &feature, double fitQualityCoeff);
        
        double computePlaneQualityMetrics(const std::vector<Point>& nearest_pts,
                                          Eigen::Vector3d &plane_normal,
                                          double &negative_OA_dot_norm,
                                          OptimizationParameter &result);

        void computeEigenProperties(pcaFeature &feature, const Eigen::Vector3d &eigenvalues);

        void computeCrossProducts(pcaFeature &feature, const RotatedAxes &axes);

        RotatedAxes computeRotatedAxes();

        void computeTranslationObservability(pcaFeature &feature, const RotatedAxes &axes);

        void analyzeFeatureObservability(pcaFeature &feature);

        void EstimateLidarUncertainty();

        void publishUncertainty(double uncer_x, double uncer_y, double uncer_z,
            double uncer_roll, double uncer_pitch, double uncer_yaw);
        
        bool validateNeighborSearch( bool found,const std::vector<Point> &nearest_pts,
        const std::vector<float> &nearest_dist, OptimizationParameter &result);

        OptimizationParameter processLineResults(
                                  const Eigen::Vector3d &pInit,
                                  const Eigen::Vector3d &mean,
                                  const Eigen::Vector3d &eigenvalues,
                                  const Eigen::Matrix3d &eigenvectors,
                                  const std::vector<Point> &nearest_pts,
                                  double square_max_dist);



        

    };      // class lidarslam
}
#endif  // LIDARSLAM_H
