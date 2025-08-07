
#include "lidar_inertial_odometry/fusion_filter/kalman_filter.h"

namespace rthpva {
namespace localization {

KalmanFilter::KalmanFilter() {}

void KalmanFilter::setStateTranslationMatrix(
    const std::unique_ptr<InertialNavigation>& inertial_navigation) {}

void KalmanFilter::stateUpdate(double time_interval) {}

bool KalmanFilter::lidarMeasureUpdate(
    const LidarCorrespondenceManager& lidar_correspondence_manager,
    bool add_p2line_constraints) {
  const std::vector<LidarCorrespondenceManager::Vector3dTuple3>
      matched_p2plane = lidar_correspondence_manager.getMatchedP2Plane();

  const size_t matched_p2plane_num = matched_p2plane.size();
  for (size_t i = 0; i < matched_p2plane_num; ++i) {
    const Vector3d point = std::get<0>(matched_p2plane[i]);
    const Vector3d plane_point = std::get<1>(matched_p2plane[i]);
    const Vector3d plane_normal = std::get<2>(matched_p2plane[i]);

    Eigen::VectorXd residual = plane_normal.dot(point - plane_point);

    Eigen::MatrixXd coefficient_matrix;
    coefficient_matrix.resize(1, kStateDimension);
    coefficient_matrix.setZero();

    Eigen::MatrixXd measure_noise;
  }

  if (add_p2line_constraints) {
    const std::vector<LidarCorrespondenceManager::Vector3dTuple3>
        matched_p2line = lidar_correspondence_manager.getMatchedP2Line();
  }

  return true;
}

void KalmanFilter::stateFeedback(InertialNavigation* inertial_navigation) {}

}  // namespace localization
}  // namespace rthpva