#pragma once

#include "common/eigen_types.h"

namespace rthpva {
namespace common {

static Eigen::Matrix<double, 3, 2> derivativeS2(const Vector3d& gravity_in) {
  Eigen::Matrix<double, 3, 2> B_x;
  Eigen::Matrix<double, 3, 1> gravity = gravity_in;

  gravity.normalize();
  B_x(0, 0) = 1.0 - gravity(0) * gravity(0) / (1.0 + gravity(2));
  B_x(0, 1) = -gravity(0) * gravity(1) / (1.0 + gravity(2));
  B_x(1, 0) = B_x(0, 1);
  B_x(1, 1) = 1.0 - gravity(1) * gravity(1) / (1.0 + gravity(2));
  B_x(2, 0) = -gravity(0);
  B_x(2, 1) = -gravity(1);

  return B_x;
}

}  // namespace common
}  // namespace rthpva