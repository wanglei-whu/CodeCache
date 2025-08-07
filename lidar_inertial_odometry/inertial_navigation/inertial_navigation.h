#pragma once

#include "common/eigen_types.h"
#include "common/s2.h"
#include "common/skew_symmetric_matrix.h"
#include "common/so3.h"
#include "common/gravity_s2.h"

#include "lidar_inertial_odometry/fusion_filter/kalman_filter.h"

namespace rthpva {
namespace localization {

class InertialNavigation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Parameter {
    double gyro_variance = 0.01;      // Gyroscope noise variance
    double acce_variance = 0.01;      // Accelerometer noise variance
    double gyro_bias_drift = 0.0001;  // Gyroscope bias drift
    double acce_bias_drift = 0.0001;  // Accelerometer bias drift
    double gravity_variance = 0.01;   // Gravity variance
  };

  InertialNavigation(const Parameter& Parameter);
  ~InertialNavigation() = default;

  void setZeroBias(const Vector3d& gyro_zero_bias,
                   const Vector3d& acce_zero_bias);

  void initialize(const Eigen::Quaterniond& attitude_quaternion,
                  const Vector3d& velocity, const Vector3d& position);

  void propagate(const Vector3d& gyro, const Vector3d& acce,
                 double time_interval);

  KalmanFilter::FilterMatrix computeStateTransitionMatrix(
      double time_interval) const;

  KalmanFilter::FilterMatrix computeProcessMatrix(double time_interval) const;

  void feedback(const Vector3d& attitude_increment,
                const Vector3d& velocity_increment,
                const Vector3d& position_increment,
                const Vector3d& gyro_bias_increment,
                const Vector3d& acce_bias_increment,
                const common::S2& gravity_increment);

 private:
  Parameter parameter_;

  Vector3d previous_gyro_;
  Vector3d previous_acce_;

  Eigen::Quaterniond previous_attitude_quaternion_;

  Eigen::Quaterniond attitude_quaternion_;
  Vector3d velocity_;
  Vector3d position_;
  Vector3d gyro_zero_bias_;
  Vector3d acce_zero_bias_;
  Vector3d gravity_;

  Vector3d compensated_gyro_;
  Vector3d compensated_acce_;
};

}  // namespace localization
}  // namespace rthpva