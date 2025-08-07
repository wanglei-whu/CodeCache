// Copyright 2022. All Rights Reserved.
// Author: Lei wang
#pragma once

#include "common/so3.h"

namespace rthpva {
namespace common {

class SE3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int kDimension = 6;
  SE3();
  SE3(const SO3& so3, const Vector3d& translation);
  SE3(const Eigen::Quaterniond& q, const Vector3d& translation);
  SE3(const Matrix4d& matrix);

  static SE3 FromEulerYPRAndTranslation(const Vector3d& euler_ypr,
                                        const Vector3d& translation);
  SO3 GetSO3() const { return so3_; }
  Vector3d GetTranslation() const { return translation_; }
  Matrix4d GetTransformationMatrix() const;
  Vector6d Log() const;
  static SE3 Exp(const Vector6d& tf);

  void SetSO3(const SO3& so3) { so3_ = so3; }
  void SetTranslation(const Vector3d& tran) { translation_ = tran; }

  SE3 operator*(const SE3& other) const;
  SE3 Compose(const SE3& other) const;
  SE3 Compose(const SE3& other, Matrix6d* J_this, Matrix6d* J_other) const;
  SE3 InverseCompose(const SE3& other) const;
  SE3 InverseCompose(const SE3& other, Matrix6d* J_this,
                     Matrix6d* J_other) const;
  SE3 Rplus(const Vector6d& tf_vec) const;
  SE3 Rplus(const Vector6d& tf_vec, Matrix6d* J_this, Matrix6d* J_other) const;
  Vector6d Rminus(const SE3& other) const;
  Vector6d Rminus(const SE3& other, Matrix6d* J_this, Matrix6d* J_other) const;
  Vector3d Transform(const Vector3d& pt) const;
  Vector3d Transform(const Vector3d& pt, Eigen::Matrix<double, 3, 6>* J_pose,
                     Matrix3d* J_point) const;
  Vector3d InverseTransform(const Vector3d& pt) const;
  Vector3d InverseTransform(const Vector3d& pt,
                            Eigen::Matrix<double, 3, 6>* J_pose,
                            Matrix3d* J_point) const;
  SE3 Inverse() const;
  void BundlePlusUpdate(const Vector6d& delta);
  SE3 BundlePlus(const Vector6d& delta) const;
  Vector6d BundleMinus(const SE3& other) const;
  Vector6d BundleMinus(const SE3& other, Matrix6d* J_this,
                       Matrix6d* J_other) const;

 private:
  SO3 so3_;
  Vector3d translation_;
};
}  // namespace common
}  // namespace rthpva
