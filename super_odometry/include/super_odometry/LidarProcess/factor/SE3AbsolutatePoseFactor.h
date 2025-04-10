//
// Created by shibo zhao on 2020-09-27.
//

#ifndef SE3ABSOLUTATEPOSEFACTOR_H
#define SE3ABSOLUTATEPOSEFACTOR_H

#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include "super_odometry/utils/EigenTypes.h"
#include "super_odometry/utils/Twist.h"
#include <Eigen/Dense>

class SE3AbsolutatePoseFactor : public ceres::SizedCostFunction<6, 7> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SE3AbsolutatePoseFactor(const Transformd &T_w_i_meas,
                          const Eigen::Mat66d &information)
      : T_w_i_meas_(T_w_i_meas), information_matrix(information) {
    sqrt_information_ =
        Eigen::LLT<Eigen::Matrix<double, 6, 6>>(information_matrix)
            .matrixL()
            .transpose();
  }
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const;
public:
  const Transformd T_w_i_meas_;

  Eigen::Mat66d information_matrix;
  Eigen::Mat66d sqrt_information_;
};

#endif // SE3ABSOLUTATEPOSEFACTOR_H
