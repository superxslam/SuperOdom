//
// Created by ubuntu on 2020/4/2.
//

#ifndef TWIST_H
#define TWIST_H

#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <super_odometry/utils/EigenTypes.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

////////////////////////////////////////////////////////////////////////////
// Forward Declarations / typedefs
////////////////////////////////////////////////////////////////////////////

typedef pcl::PointXYZI PointType;
template <typename T> struct Twist;

typedef Twist<float> Transformf;

typedef Twist<double> Transformd;

namespace TwistConstants {
template <typename Scalar> struct Constants {
  EIGEN_ALWAYS_INLINE static const Scalar epsilon() {
    return static_cast<Scalar>(1e-10);
  }

  EIGEN_ALWAYS_INLINE static const Scalar pi() {
    return static_cast<Scalar>(M_PI);
  }
};

template <> struct Constants<float> {
  EIGEN_ALWAYS_INLINE static float epsilon() {
    return static_cast<float>(1e-5);
  }

  EIGEN_ALWAYS_INLINE static float pi() { return static_cast<float>(M_PI); }
};
} // namespace TwistConstants

template <typename T> struct Twist {
public:
  typedef Eigen::Matrix<T, 4, 4> Transformation;
  typedef Eigen::Matrix<T, 3, 3> Matrix3x3;
  typedef Eigen::Matrix<T, 6, 6> Adjoint;
  typedef Eigen::Matrix<T, 6, 1> Tangent;
  typedef Eigen::Matrix<T, 3, 1> Tangent3;
  typedef Eigen::Matrix<T, 4, 1> HomogeneousPoint;

public:
public:
  Eigen::Quaternion<T> rot;
  Eigen::Matrix<T, 3, 1> pos;

public:
  static Twist Identity() { return Twist(); }

  Twist() : rot(T(1), T(0), T(0), T(0)), pos(Tangent3::Zero()) {}

  Twist(const Eigen::Quaternion<T> &rot_in,
        const Eigen::Matrix<T, 3, 1> &pos_in)
      : rot(rot_in), pos(pos_in) {}

  Twist(const Transformation &matrix)
      : rot(matrix.template topLeftCorner<3, 3>()),
        pos(matrix.template block<3, 1>(0, 3)) {}

  Twist(const Matrix3x3 &R, const Tangent3 &t) : rot(R), pos(t) {}

  Twist(
      const Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> &transform) {
    this->rot = Eigen::Quaternion<T>{transform.linear()}.normalized();
    this->pos = transform.translation();
  }

  Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform() const {
    Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform;
    transform.linear() = rot.normalized().toRotationMatrix();
    transform.translation() = pos;
    return transform;
  }

  inline Transformation matrix() const {
    Transformation homogenious_matrix;
    homogenious_matrix.setIdentity();
    homogenious_matrix.block(0, 0, 3, 3) =
        this->rot.normalized().toRotationMatrix();
    homogenious_matrix.col(3).head(3) = this->pos;

    return homogenious_matrix;
  }

  inline Eigen::Matrix<T, 3, 4> matrix3x4() const {
    Eigen::Matrix<T, 3, 4> matrix;
    matrix.block(0, 0, 3, 3) = this->rot.normalized().toRotationMatrix();
    matrix.col(3) = this->pos;

    return matrix;
  }

  inline Matrix3x3 rotationMatrix() const {
    Matrix3x3 matrix;
    matrix = this->rot.normalized().toRotationMatrix();

    return matrix;
  }

  inline T *data() { return rot.coeffs().data(); }

  // Const version of data() above.
  T const *data() const { return rot.coeffs().data(); }

  inline Adjoint SE3Adj() const {
    const Eigen::Matrix<T, 3, 3> R = this->rot.normalized().toRotationMatrix();
    Adjoint res;
    res.template block<3, 3>(0, 0) = R;
    res.template block<3, 3>(3, 3) = R;
    res.template block<3, 3>(0, 3) = hat(this->pos) * R;
    res.template block<3, 3>(3, 0) = Eigen::Matrix<T, 3, 3>::Zero(3, 3);

    return res;
  }

  inline Adjoint SO3TransAdj() const {
    const Eigen::Matrix<T, 3, 3> R = this->rot.normalized().toRotationMatrix();
    Adjoint res;
    res.template block<3, 3>(0, 0) = R;
    res.template block<3, 3>(3, 3) = R;
    res.template block<3, 3>(0, 3) = hat(this->pos) * R + R * hat(this->pos);
    res.template block<3, 3>(3, 0) = Eigen::Matrix<T, 3, 3>::Zero(3, 3);

    return res;
  }

  inline static Twist<T> se3exp(const Tangent &a) {
    const Eigen::Matrix<T, 3, 1> &omega = a.template tail<3>();

    T theta;
    const Eigen::Quaternion<T> &so3 = expAndTheta(omega, &theta);
    const Eigen::Matrix<T, 3, 3> &Omega = hat(omega);
    const Eigen::Matrix<T, 3, 3> &Omega_sq = Omega * Omega;
    Eigen::Matrix<T, 3, 3> V;

    if (theta < TwistConstants::Constants<T>::epsilon()) {
      // Note: That is an accurate expansion!
      V = so3.matrix();
    } else {
      T theta_sq = theta * theta;
      V = (Eigen::Matrix<T, 3, 3>::Identity() +
           (static_cast<T>(1) - std::cos(theta)) / (theta_sq)*Omega +
           (theta - std::sin(theta)) / (theta_sq * theta) * Omega_sq);
    }

    return Twist<T>(so3, V * a.template head<3>());
  }

  inline static Twist<T> so3Transexp(const Tangent &a) {
    const Eigen::Matrix<T, 3, 1> &t = a.template head<3>();
    const Eigen::Matrix<T, 3, 1> &omega = a.template tail<3>();
    T theta;
    const Eigen::Quaternion<T> &so3 = expAndTheta(omega, &theta);

    return Twist<T>(so3, t);
  }

  Twist inverse() const {
    //        Eigen::Transform<T, 3, Eigen::TransformTraits::Affine>
    //        transform_inv = this->transform().inverse();
    Twist twist_inv;
    twist_inv.rot = this->rot.conjugate();
    twist_inv.pos = -twist_inv.rot.toRotationMatrix() * this->pos;
    return twist_inv;
  }

  Twist operator*(const Twist &other) const {
    Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform_out =
        this->transform() * other.transform();
    return Twist(transform_out);
  }

  Tangent3 operator*(const Tangent3 &p) const { return rot * p + pos; }

  HomogeneousPoint operator*(const HomogeneousPoint &p) const {
    const Tangent3 tp = rot * p.template head<3>() + p(3) * pos;
    return HomogeneousPoint(tp(0), tp(1), tp(2), p(3));
  }

  template <typename NewType> Twist<NewType> cast() const {
    Twist<NewType> twist_new{this->rot.template cast<NewType>(),
                             this->pos.template cast<NewType>()};
    return twist_new;
  }

  friend std::ostream &operator<<(std::ostream &os, const Twist &twist) {
    os << twist.pos.x() << " " << twist.pos.y() << " " << twist.pos.z() << " "
       << twist.rot.w() << " " << twist.rot.x() << " " << twist.rot.y() << " "
       << twist.rot.z();
    return os;
  }

  inline static Eigen::Quaternion<T> expAndTheta(const Tangent3 &omega,
                                                 T *theta) {
    const T theta_sq = omega.squaredNorm();
    *theta = std::sqrt(theta_sq);
    const T half_theta = static_cast<T>(0.5) * (*theta);

    T imag_factor;
    T real_factor;
    ;
    if ((*theta) < TwistConstants::Constants<T>::epsilon()) {
      const T theta_po4 = theta_sq * theta_sq;
      imag_factor = static_cast<T>(0.5) -
                    static_cast<T>(1.0 / 48.0) * theta_sq +
                    static_cast<T>(1.0 / 3840.0) * theta_po4;
      real_factor = static_cast<T>(1) - static_cast<T>(0.5) * theta_sq +
                    static_cast<T>(1.0 / 384.0) * theta_po4;
    } else {
      const T sin_half_theta = std::sin(half_theta);
      imag_factor = sin_half_theta / (*theta);
      real_factor = std::cos(half_theta);
    }

    return Eigen::Quaternion<T>(real_factor, imag_factor * omega.x(),
                                imag_factor * omega.y(),
                                imag_factor * omega.z());
  }

private:
  inline static const Matrix3x3 hat(const Tangent3 &omega) {
    Matrix3x3 Omega;
    Omega << static_cast<T>(0), -omega(2), omega(1), omega(2),
        static_cast<T>(0), -omega(0), -omega(1), omega(0), static_cast<T>(0);
    return Omega;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // class Twist

#endif // TWIST_H
