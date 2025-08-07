
#pragma once

#include "lidar_inertial_odometry/fusion_filter/kalman_filter.h"
#include "lidar_inertial_odometry/inertial_navigation/inertial_navigation.h"
#include "lidar_inertial_odometry/lidar_odometry/lidar_odometry.h"

namespace rthpva {
namespace localization {

class LidarInertialOdometry {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LidarInertialOdometry();

  ~LidarInertialOdometry() = default;

  void handleRawimu();

  void handleLidarPointcloud();

 private:
  std::unique_ptr<InertialNavigation> inertial_navigation_;

  std::unique_ptr<LidarOdometry> lidar_odometry_;

  std::unique_ptr<KalmanFilter> fusion_filter_;
};

}  // namespace localization
}  // namespace rthpva