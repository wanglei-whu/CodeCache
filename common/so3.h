#pragma once

#include <Eigen/Geometry>
#include "common/eigen_types.h"

namespace rthpva {
namespace common {

class SO3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int kDimension = 3;
  SO3();
  SO3(const Eigen::Quaterniond& q);

  bool IsValid() const;
  // 2019.03.28: remove SO3 ctor form euler angle since it is confused with
  // rotation vector.
  static SO3 FromEulerYPR(const Vector3d& euler_anlge_ypr);
  // `fromRotationVector` is a simple wrapper of `exp`.
  static SO3 FromRotationVector(const Vector3d& rot_vec);
  static bool ConvertEulerYPRToSO3(const Vector3d& yaw_pitch_roll, SO3* rot3);
  static bool ConvertRotationMatrixToSO3(const Matrix3d& R, SO3* so3);
  static Matrix3d ConvertEulerYPRToRotationMatrix(
      const Vector3d& yaw_pitch_roll);
  static Vector3d ConvertRotationMatrixToEulerYPR(const Matrix3d& R);
  static Eigen::AngleAxisd ConvertQuaternionToAngleAxis(
      const Eigen::Quaterniond& q);

  inline Matrix3d GetRotationMatrix() const {
    return quaternion_.toRotationMatrix();
  };
  Vector3d GetEulerYPR() const;
  // `J_this` is `d(ypr)/d(disturb to this SE3)`.
  Vector3d GetEulerYPR(Matrix3d* J_this) const;
  Vector4d GetQuaternionWxyz() const;
  Eigen::Quaterniond GetQuaternion() const { return quaternion_; };

  static SO3 Exp(const Vector3d& rotation_vec);
  Vector3d Log() const;
  SO3 operator*(const SO3& other) const;
  void RplusUpdate(const Vector3d& rot_vec);
  SO3 Rplus(const Vector3d& rot_vec) const;
  // `this` rminus `other` = log(other.inverse * this);
  Vector3d Rminus(const SO3& other) const;
  SO3 Rminus(const Vector3d& rotation_vector) const;
  Vector3d Rminus(const SO3& other, Matrix3d* J_this, Matrix3d* J_other) const;
  static Matrix3d RightJacobian(const SO3& so3);
  static Matrix3d RightJacobian(const Eigen::AngleAxisd& angle_axis);
  static Matrix3d RightJacobianInv(const Eigen::AngleAxisd& angle_axis);
  static Matrix3d RightJacobianInv(const SO3& so3);

  Vector3d operator*(const Vector3d& point) const;
  Vector3d Rotate(const Vector3d& point) const;
  Vector3d InverseRotate(const Vector3d& point) const;
  inline SO3 Inverse() const { return SO3(quaternion_.inverse()); };

 private:
  inline void NormalizeQuaternion() { quaternion_.normalize(); };
  Eigen::Quaterniond quaternion_;
};
}  // namespace common
}  // namespace rthpva
