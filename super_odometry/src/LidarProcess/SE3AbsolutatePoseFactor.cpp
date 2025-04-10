//
// Created by ubuntu on 2020/7/7.
//

#include "super_odometry/LidarProcess/factor/SE3AbsolutatePoseFactor.h"
#include "super_odometry/utils/sophus_utils.hpp"
#include "super_odometry/utils/utility.h"

bool SE3AbsolutatePoseFactor::Evaluate(double const *const *parameters,
                                       double *residuals,
                                       double **jacobians) const {
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                        parameters[0][5]);

  Transformd T_w_i_estimate(Qi, Pi);

  Eigen::Map<Sophus::Vector6d> res(residuals);

  // residual p
  res.segment<3>(0) = T_w_i_estimate.pos - T_w_i_meas_.pos;

  // residual R
  Eigen::Vector3d error_quat = 2.0 * (T_w_i_meas_.rot.conjugate() * Qi).vec();

  res.segment<3>(3) = error_quat;

  res.applyOnTheLeft(sqrt_information_);

  if (jacobians) {

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_from(
          jacobians[0]);
      jacobian_from.setZero();

      // res_p_dpi
      jacobian_from.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

      // dr_dRi
      jacobian_from.block<3, 3>(3, 3) =
          (Utility::Qleft(T_w_i_meas_.rot.conjugate() * Qi))
              .bottomRightCorner<3, 3>();

      // LOG(ERROR) << "\ndR_dqi:\n" << dR_dqi;

      jacobian_from.applyOnTheLeft(sqrt_information_);
    }
  }
  return true;
}
