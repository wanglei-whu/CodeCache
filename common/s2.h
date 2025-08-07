#pragma once

#include "common/eigen_types.h"
#include "common/so3.h"

namespace rthpva {
namespace common {

class S2 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit S2(const Vector3d& gravity);
  ~S2() = default;

  S2 operator+(const Vector2d& delta) const;

  Vector2d operator-(const S2& other) const;

  Eigen::Matrix<double, 3, 2> project() const;

  inline const Vector3d& gravity() const { return gravity_; }

 private:
  Vector3d gravity_;
};

}  // namespace common
}  // namespace rthpva
