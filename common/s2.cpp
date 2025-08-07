#include "common/s2.h"

namespace rthpva {
namespace common {

S2::S2(const Vector3d& gravity) : gravity_(gravity) {}

S2::operator+(const Vector2d& delta) const {
  return SO3::Exp(project() * delta) * gravity_;
}

S2::operator-(const S2& other) const {
  const Vector3d gravity_0 = this->gravity_.normalized();
  const Vector3d gravity_1 = other.gravity_.normalized();

  const Vector3d rotation_vector =
      SO3(Eigen::Quaterniond::FromTwoVectors(gravity_0, gravity_1)
              .toRotationMatrix())
          .Log();
  return other.project().transpose() * rotation_vector;
}

Eigen::Matrix<double, 3, 2> S2::project() const {
  Eigen::Matrix<double, 3, 2> projection;
  projection.setZero();
  const Vector3d normalized_gravity = gravity_.normalized();

  projection(0, 0) = 1.0 - normalized_gravity(0) * normalized_gravity(0) /
                               (1.0 + normalized_gravity(2));
  projection(0, 1) = -normalized_gravity(0) * normalized_gravity(1) /
                     (1.0 + normalized_gravity(2));
  projection(1, 0) = -normalized_gravity(1) * normalized_gravity(0) /
                     (1.0 + normalized_gravity(2));
  projection(1, 1) = 1.0 - normalized_gravity(1) * normalized_gravity(1) /
                               (1.0 + normalized_gravity(2));
  projection(2, 0) = -normalized_gravity(0);
  projection(2, 1) = -normalized_gravity(1);

  return projection;

}  // namespace common
}  // namespace rthpva