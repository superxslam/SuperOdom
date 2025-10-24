//
// Created by ubuntu on 2020/6/29.
//

#ifndef IMU_DATA_H
#define IMU_DATA_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include "super_odometry/container/MapRingBuffer.h"
#include <sensor_msgs/msg/imu.hpp>
#include "super_odometry/config/parameter.h"

struct Imu {

public:
  typedef std::shared_ptr<Imu> Ptr;
  typedef std::shared_ptr<const Imu> ConstPtr;

public:
  Imu() : Imu(0.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)) {}
  Imu(double time, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
      : time(time), acc(acc), gyr(gyr) 
      {
        initialize();
      }
  ~Imu() {}
  
  void initialize()
  {
    acc_mean<<0.0,0.0,-1.0;
    gyr_mean<<0.0,0.0,0.0;
    acc_cov<<0.1,0.1,0.1;
    gyr_cov<<0.1,0.1,0.1;
    gravity<<0.0,0.0,0.0;
    gyr_bias<<0.0,0.0,0.0;
    gyr_bias<<0.0,0.0,0.0;
    q_w_i.setIdentity();
  }

  // Function to calculate pitch and roll and construct the combined rotation matrix
  Eigen::Matrix3d calculatePitchRollMatrix(double ax, double ay, double az) {
      // Calculate the pitch angle (theta)
      double theta = std::atan2(ax, std::sqrt(ay * ay + az * az));

      // Calculate the roll angle (phi)
      double phi = std::atan2(-ay, az);

      // Construct the pitch rotation matrix
      Eigen::Matrix3d R_y;
      R_y << std::cos(theta), 0, std::sin(theta),
            0, 1, 0,
            -std::sin(theta), 0, std::cos(theta);

      // Construct the roll rotation matrix
      Eigen::Matrix3d R_x;
      R_x << 1, 0, 0,
            0, std::cos(phi), -std::sin(phi),
            0, std::sin(phi), std::cos(phi);
      
      // Combine the rotation matrices
      Eigen::Matrix3d R = R_x * R_y;
      pitch_offset_gravity=theta;
      roll_offset_gravity=phi;
      return R;
  }

  void imuInit(MapRingBuffer<Imu::Ptr> imuBuf) {
      
    
      int Num = 0;
      if (first_imu==false){
          return;
      }
      
    
      
      // Initialize if the buffer is not empty
      if (!imuBuf.empty()) {
          first_imu = false;
          const double &time_first = imuBuf.measMap_.begin()->second->time;
          const Eigen::Quaterniond rot_first = imuBuf.measMap_.begin()->second->q_w_i;
          const Eigen::Vector3d gyr_first = imuBuf.measMap_.begin()->second->gyr;
          const Eigen::Vector3d acc_first = imuBuf.measMap_.begin()->second->acc;
          acc_mean = acc_first;
          gyr_mean = gyr_first;
          time = time_first;
        
          Num = 1;
      }
    
      // Iterate through the IMU buffer and update mean and covariance
      for (std::map<double, Imu::Ptr>::iterator itMeas_ = imuBuf.measMap_.begin(); itMeas_ != imuBuf.measMap_.end(); ++itMeas_) {

          const double &time_cur = itMeas_->second->time;
          const Eigen::Quaterniond rot_cur = itMeas_->second->q_w_i;
          const Eigen::Vector3d gyr_cur = itMeas_->second->gyr;
          const Eigen::Vector3d acc_cur = itMeas_->second->acc;
          
        
          // Update means
          acc_mean += (acc_cur - acc_mean) / Num;
          gyr_mean += (gyr_cur - gyr_mean) / Num;

          // Update covariances
          acc_cov = acc_cov * (Num - 1.0) / Num + (acc_cur - acc_mean).cwiseProduct(acc_cur - acc_mean) / (Num - 1.0);
          gyr_cov = gyr_cov * (Num - 1.0) / Num + (gyr_cur - gyr_mean).cwiseProduct(gyr_cur - gyr_mean) / (Num - 1.0);
          Num++;
      }
    

      //TODO: double check the gravity direction
      gravity=  acc_mean / acc_mean.norm() *gravity_norm;
      gyr_bias = gyr_mean;
      acc_bias = acc_mean;
      first_imu = false;

      // //Align with Gravity if the IMU is rotated at the beginning. 
      Roll_Pitch_Gravity_Matrix=calculatePitchRollMatrix(acc_mean.x(), 
      acc_mean.y(), acc_mean.z());


      imu_laser_R_Gravity=Roll_Pitch_Gravity_Matrix.inverse()*imu_laser_R;
          
        
    
      std::cout<<"IMU Data Summary"<<std::endl;
      std::cout<<"Gravity: "<<gravity.transpose()<<std::endl;
      std::cout<<"Gyroscope Bias: "<<gyr_bias.transpose()<<std::endl;
      std::cout<<"Accelerometer Bias: "<<acc_bias.transpose()<<std::endl;
      std::cout<<"Accelerometer Mean: "<<acc_mean.transpose()<<std::endl;
      std::cout<<"pitch offset gravity: "<<pitch_offset_gravity*180/M_PI<<std::endl;
      std::cout<<"roll offset gravity: "<<roll_offset_gravity*180/M_PI<<std::endl;
      std::cout<<"Roll Pitch Gravity Matrix: "<<Roll_Pitch_Gravity_Matrix<<std::endl;
      std::cout<<"Imu laser R Gravity: "<<imu_laser_R_Gravity<<std::endl;
      saveImuLaserRGravity("imu_laser_R_Gravity.txt");

      
  }


  void saveImuLaserRGravity(const std::string& filename) const {
    std::ofstream file(filename);
    if (file.is_open()) {
      file << imu_laser_R_Gravity(0, 0) << ", " << imu_laser_R_Gravity(0, 1) << ", " << imu_laser_R_Gravity(0, 2) << ",\n"
           << imu_laser_R_Gravity(1, 0) << ", " << imu_laser_R_Gravity(1, 1) << ", " << imu_laser_R_Gravity(1, 2) << ",\n"
           << imu_laser_R_Gravity(2, 0) << ", " << imu_laser_R_Gravity(2, 1) << ", " << imu_laser_R_Gravity(2, 2) << ";\n";
      file.close();
    } else {
      std::cerr << "Unable to open file: " << filename << std::endl;
    }
  }

  // Function to convert a rotation matrix to roll, pitch, and yaw
  Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) {
    Eigen::Vector3d rpy;

    // Extract the roll, pitch, and yaw from the rotation matrix
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));

    bool singular = sy < 1e-6; // If

    if (!singular) {
        rpy(0) = atan2(R(2,1), R(2,2)); // Roll
        rpy(1) = atan2(-R(2,0), sy);     // Pitch
        rpy(2) = atan2(R(1,0), R(0,0));  // Yaw
    } else {
        rpy(0) = atan2(-R(1,2), R(1,1)); // Roll
        rpy(1) = atan2(-R(2,0), sy);     // Pitch
        rpy(2) = 0;                      // Yaw
    }

    return rpy;
}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool first_imu = true;
  double time;
  double gravity_norm = 9.8105;
  double imu_frequency = 200.0; // default 200 Hz
  Eigen::Vector3d gravity;
  Eigen::Vector3d gyr_bias;
  Eigen::Vector3d acc_bias;

  Eigen::Vector3d acc_mean; // mean of accelerometer measurement (m^2/sec)
  Eigen::Vector3d gyr_mean; // mean of gyroscope measurement (rad/s)
  Eigen::Vector3d acc; // accelerometer measurement (m^2/sec)
  Eigen::Vector3d gyr; // gyroscope measurement (rad/s)
  Eigen::Vector3d acc_cov; // covariance of accelerometer measurement
  Eigen::Vector3d gyr_cov; // covariance of gyroscope measurement
  Eigen::Quaterniond q_w_i;
  Eigen::Matrix3d Roll_Pitch_Gravity_Matrix;
  Eigen::Matrix3d imu_laser_R_Gravity;

  double pitch_offset_gravity;
  double roll_offset_gravity;
};

#endif // IMU_DATA_H
