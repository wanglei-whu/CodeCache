
#include "lidar_inertial_odometry/inertial_navigation/inertial_navigation.h"

namespace rthpva {
namespace localization {

InertialNavigation::InertialNavigation(const Parameter& parameter)
    : parameter_(parameter),
      attitude_quaternion_(Eigen::Quaterniond::Identity()),
      velocity_(Vector3d::Zero()),
      position_(Vector3d::Zero()),
      gyro_zero_bias_(Vector3d::Zero()),
      acce_zero_bias_(Vector3d::Zero()),
      gravity_(Vector3d(0, 0, 9.79)) {}

void InertialNavigation::setZeroBias(const Vector3d& gyro_zero_bias,
                                     const Vector3d& acce_zero_bias) {
  gyro_zero_bias_ = gyro_zero_bias;
  acce_zero_bias_ = acce_zero_bias;
}

void InertialNavigation::initialize(
    const Eigen::Quaterniond& attitude_quaternion, const Vector3d& velocity,
    const Vector3d& position) {
  attitude_quaternion_ = attitude_quaternion;
  velocity_ = velocity;
  position_ = position;
}

void InertialNavigation::propagate(const Vector3d& gyro, const Vector3d& acce,
                                   double time_interval) {
  compensated_gyro_ = 0.5 * (gyro + previous_gyro_) - gyro_zero_bias_;
  compensated_acce_ = 0.5 * (acce + previous_acce_) - acce_zero_bias_;

  previous_attitude_quaternion_ = attitude_quaternion_;

  const Vector3d global_acce =
      attitude_quaternion_ * compensated_acce_ - gravity_;

  attitude_quaternion_ =
      attitude_quaternion_ *
      common::SO3::FromRotationVector(compensated_gyro_ * time_interval)
          .GetQuaternion();
  position_ = position_ + velocity_ * time_interval +
              0.5 * global_acce * time_interval * time_interval;
  velocity_ = velocity_ + global_acce * time_interval;
}

KalmanFilter::FilterMatrix InertialNavigation::computeStateTransitionMatrix(
    double time_interval) const {
  KalmanFilter::FilterMatrix state_transition_matrix =
      KalmanFilter::FilterMatrix::Zero();

  const Matrix3d gyro_skem_symmetrix_matrix =
      common::SkewSymmetricMatrix<double>(compensated_gyro_);
  const Matrix3d acce_skem_symmetrix_matrix =
      common::SkewSymmetricMatrix<double>(compensated_acce_);

  Eigen::Matrix<double, 3, 2> B_x = common::derivativeS2(gravity_);

  state_transition_matrix.block<3, 3>(0, 0) =
      Matrix3d::Identity() - gyro_skem_symmetrix_matrix * time_interval;
  state_transition_matrix.block<3, 3>(0, 9) =
      -Matrix3d::Identity() * time_interval;

  state_transition_matrix.block<3, 3>(3, 0) =
      -previous_attitude_quaternion_.toRotationMatrix() *
      acce_skem_symmetrix_matrix * time_interval;
  state_transition_matrix.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  state_transition_matrix.block<3, 3>(3, 12) =
      -previous_attitude_quaternion_.toRotationMatrix() * time_interval;
  state_transition_matrix.block<3, 2>(3, 21) =
      common::SkewSymmetricMatrix<double>(gravity_) * B_x * time_interval;

  state_transition_matrix.block<3, 3>(6, 3) =
      Eigen::Matrix3d::Identity() * time_interval;
  state_transition_matrix.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
  state_transition_matrix.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
  state_transition_matrix.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();

  state_transition_matrix.block<2, 2>(21, 21) =
      -1.0 / (gravity_.norm() * gravity_.norm()) * B_x.transpose() *
      common::SkewSymmetricMatrix<double>(gravity_) *
      common::SkewSymmetricMatrix<double>(gravity_) * B_x;

  return state_transition_matrix;
}

KalmanFilter::FilterMatrix InertialNavigation::computeProcessMatrix(
    double time_interval) const {
  Eigen::Matrix<double, 23, 12> process_matrix = Eigen::MatrixXd::Zero(23, 12);

  process_matrix.block<3, 3>(6, 0) =
      -previous_attitude_quaternion_.toRotationMatrix() * time_interval;
  process_matrix.block<3, 3>(3, 3) =
      -Eigen::Matrix3d::Identity() * time_interval;
  process_matrix.block<3, 3>(9, 6) =
      -Eigen::Matrix3d::Identity() * time_interval;
  process_matrix.block<3, 3>(12, 9) =
      -Eigen::Matrix3d::Identity() * time_interval;

  return process_matrix;
}
}  // namespace localization
}  // namespace rthpva
