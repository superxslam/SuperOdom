//
// Created by ubuntu on 2020/5/23.
//

#ifndef SOPHUS_UTILS_HPP
#define SOPHUS_UTILS_HPP

#include "sophus/se3.hpp"
namespace Sophus {

template <typename Scalar>
inline static typename SE3<Scalar>::Tangent logd(const SE3<Scalar> &se3) {
    typename SE3<Scalar>::Tangent upsilon_omega;
    upsilon_omega.template tail<3>() = se3.so3().log();
    upsilon_omega.template head<3>() = se3.translation();

    return upsilon_omega;
}

template <typename Derived>
inline static SE3<typename Derived::Scalar> expd(
    const Eigen::MatrixBase<Derived> &upsilon_omega) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6);

    using Scalar = typename Derived::Scalar;

    return SE3<Scalar>(SO3<Scalar>::exp(upsilon_omega.template tail<3>()),
                       upsilon_omega.template head<3>());
}

// exp(phi+e) ~= exp(phi)*exp(J*e)
template <typename Derived1, typename Derived2>
void rightJacobianSO3(const Eigen::MatrixBase<Derived1> &phi,
                      const Eigen::MatrixBase<Derived2> &J_const) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_const);

    Scalar phi_norm2 = phi.squaredNorm();
    Scalar phi_norm = std::sqrt(phi_norm2);
    Scalar phi_norm3 = phi_norm2 * phi_norm;

    J.setIdentity();

    if (Sophus::Constants<Scalar>::epsilon() < phi_norm) {
        Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
        Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

        J -= phi_hat * (1 - std::cos(phi_norm)) / phi_norm2;
        J += phi_hat2 * (phi_norm - std::sin(phi_norm)) / phi_norm3;
    }
}

// log(exp(phi)exp(e)) ~= phi + J*e
template <typename Derived1, typename Derived2>
void rightJacobianInvSO3(const Eigen::MatrixBase<Derived1> &phi,
                         const Eigen::MatrixBase<Derived2> &J_const) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_const);

    Scalar phi_norm2 = phi.squaredNorm();
    Scalar phi_norm = std::sqrt(phi_norm2);

    J.setIdentity();

    if (Sophus::Constants<Scalar>::epsilon() < phi_norm) {
        Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
        Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

        J += phi_hat / 2;
        J += phi_hat2 * (1 / phi_norm2 - (1 + std::cos(phi_norm)) /
            (2 * phi_norm * std::sin(phi_norm)));
    }
}

// exp(phi+e) ~= exp(J*e)*exp(phi)
template <typename Derived1, typename Derived2>
void leftJacobianSO3(const Eigen::MatrixBase<Derived1> &phi,
                     const Eigen::MatrixBase<Derived2> &J_const) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_const);

    Scalar phi_norm2 = phi.squaredNorm();
    Scalar phi_norm = std::sqrt(phi_norm2);
    Scalar phi_norm3 = phi_norm2 * phi_norm;

    J.setIdentity();

    if (Sophus::Constants<Scalar>::epsilon() < phi_norm) {
        Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
        Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

        J += phi_hat * (1 - std::cos(phi_norm)) / phi_norm2;
        J += phi_hat2 * (phi_norm - std::sin(phi_norm)) / phi_norm3;
    }
}

// log(exp(e)*exp(phi)) ~= phi + J*e
template <typename Derived1, typename Derived2>
void leftJacobianInvSO3(const Eigen::MatrixBase<Derived1> &phi,
                        const Eigen::MatrixBase<Derived2> &J_const) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_const);

    Scalar phi_norm2 = phi.squaredNorm();
    Scalar phi_norm = std::sqrt(phi_norm2);

    J.setIdentity();

    if (Sophus::Constants<Scalar>::epsilon() < phi_norm) {
        Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
        Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

        J -= phi_hat / 2;
        J += phi_hat2 * (1 / phi_norm2 - (1 + std::cos(phi_norm)) /
            (2 * phi_norm * std::sin(phi_norm)));
    }
}

// expd(phi+e) ~= expd(phi)*expd(J*e)
template <typename Derived1, typename Derived2>
void rightJacobianSE3Decoupled(const Eigen::MatrixBase<Derived1> &phi,
                               const Eigen::MatrixBase<Derived2> &J_const) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 6, 6);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_const);

    J.setZero();

    Eigen::Matrix<Scalar, 3, 1> omega = phi.template tail<3>();
    rightJacobianSO3(omega, J.template bottomRightCorner<3, 3>());
    J.template topLeftCorner<3, 3>() =
        Sophus::SO3<Scalar>::exp(omega).inverse().matrix();
}

// logd(expd(phi)expd(e)) ~= phi + J*e
template <typename Derived1, typename Derived2>
void rightJacobianInvSE3Decoupled(const Eigen::MatrixBase<Derived1> &phi,
                                  const Eigen::MatrixBase<Derived2> &J_const) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 6, 6);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_const);

    J.setZero();

    Eigen::Matrix<Scalar, 3, 1> omega = phi.template tail<3>();
    rightJacobianInvSO3(omega, J.template bottomRightCorner<3, 3>());
    J.template topLeftCorner<3, 3>() = Sophus::SO3<Scalar>::exp(omega).matrix();
}

}  // namespace Sophus

#endif //SOPHUS_UTILS_HPP