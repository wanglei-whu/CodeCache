#pragma once

#include <memory>

#include "common/eigen_types.h"
#include "lidar_inertial_odometry/lidar_odometry/lidar_correspondence_manager.h"

namespace rthpva {
namespace localization {

class InertialNavigation;

class KalmanFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  constexpr static int kStateDimension = 23;

  using FilterVector = Eigen::Matrix<double, kStateDimension, 1>;
  using FilterMatrix = Eigen::Matrix<double, kStateDimension, kStateDimension>;

  struct Parameter {};

  struct MeasureNoise {};

  KalmanFilter();

  ~KalmanFilter() = default;

  void setStateTranslationMatrix(
      const std::unique_ptr<InertialNavigation>& inertial_navigation);

  void stateUpdate(double time_interval);

  bool lidarMeasureUpdate(
      const LidarCorrespondenceManager& lidar_correspondence_manager,
      bool add_p2line_constraints = false);

  void stateFeedback(InertialNavigation* inertial_navigation);

 private:
  void stateAndStateCovarianceConstrain();

  void stateCovarianceSymmetry();

  void kalmanFilterMeasureUpdate();

  FilterVector state_;
  FilterVector max_state_;

  FilterMatrix state_translation_matrix_;

  FilterMatrix state_covariance_;
  FilterMatrix max_state_covariance_;
  FilterMatrix min_state_covariance_;

  FilterMatrix state_noise_;
  MeasureNoise measure_noise_;
};
// namespace localization
}  // namespace rthpva