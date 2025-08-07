// Copyright 2022. All Rights Reserved.
// Author: Lei wang
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace rthpva {

template <typename T, int D>
using Vector = Eigen::Matrix<T, D, 1>;
template <typename T, int D>
using Matrix = Eigen::Matrix<T, D, D>;

template <typename T>
using Vector1 = Vector<T, 1>;
template <typename T>
using Vector2 = Vector<T, 2>;
template <typename T>
using Vector3 = Vector<T, 3>;
template <typename T>
using Vector4 = Vector<T, 4>;
template <typename T>
using Vector5 = Vector<T, 5>;
template <typename T>
using Vector6 = Vector<T, 6>;

template <typename T>
using Matrix1 = Matrix<T, 1>;
template <typename T>
using Matrix2 = Matrix<T, 2>;
template <typename T>
using Matrix3 = Matrix<T, 3>;
template <typename T>
using Matrix4 = Matrix<T, 4>;
template <typename T>
using Matrix5 = Matrix<T, 5>;
template <typename T>
using Matrix6 = Matrix<T, 6>;
using MatrixXd = Eigen::MatrixXd;
using MatrixXf = Eigen::MatrixXf;

// Types for vectors.
using Vector1f = Vector1<float>;
using Vector2f = Vector2<float>;
using Vector3f = Vector3<float>;
using Vector4f = Vector4<float>;
using Vector5f = Vector5<float>;
using Vector6f = Vector6<float>;
using Vector1d = Vector1<double>;
using Vector2d = Vector2<double>;
using Vector3d = Vector3<double>;
using Vector4d = Vector4<double>;
using Vector5d = Vector5<double>;
using Vector6d = Vector6<double>;
using Vector9d = Eigen::Matrix<double, 9, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Vector18d = Eigen::Matrix<double, 18, 1>;
using Vector3i = Vector3<int>;

// Types for matrices.
using Matrix1f = Matrix1<float>;
using Matrix2f = Matrix2<float>;
using Matrix3f = Matrix3<float>;
using Matrix4f = Matrix4<float>;
using Matrix5f = Matrix5<float>;
using Matrix6f = Matrix6<float>;
using Matrix1d = Matrix1<double>;
using Matrix2d = Matrix2<double>;
using Matrix3d = Matrix3<double>;
using Matrix4d = Matrix4<double>;
using Matrix5d = Matrix5<double>;
using Matrix6d = Matrix6<double>;

// Types for vector of matrices which must align memory.
// ref: http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
using Vector2fVector = std::vector<Vector2f>;
using Vector3fVector = std::vector<Vector3f>;
using Vector4fVector =
    std::vector<Vector4f, Eigen::aligned_allocator<Vector4f>>;
using Vector5fVector = std::vector<Vector5f>;
using Vector6fVector = std::vector<Vector6f>;
using Vector2dVector =
    std::vector<Vector2d, Eigen::aligned_allocator<Vector2d>>;
using Vector3dVector = std::vector<Vector3d>;
using Vector4dVector =
    std::vector<Vector4d, Eigen::aligned_allocator<Vector4d>>;
using Vector5dVector = std::vector<Vector5d>;
using Vector6dVector =
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>>;

using Matrix2fVector =
    std::vector<Matrix2f, Eigen::aligned_allocator<Matrix2f>>;
using Matrix3fVector = std::vector<Matrix3f>;
using Matrix4fVector =
    std::vector<Matrix4f, Eigen::aligned_allocator<Matrix4f>>;
using Matrix5fVector = std::vector<Matrix5f>;
using Matrix6fVector =
    std::vector<Matrix6f, Eigen::aligned_allocator<Matrix6f>>;
using Matrix2dVector =
    std::vector<Matrix2d, Eigen::aligned_allocator<Matrix2d>>;
using Matrix3dVector = std::vector<Matrix3d>;
using Matrix4dVector =
    std::vector<Matrix4d, Eigen::aligned_allocator<Matrix4d>>;
using Matrix5dVector = std::vector<Matrix5d>;
using Matrix6dVector =
    std::vector<Matrix6d, Eigen::aligned_allocator<Matrix6d>>;

}  // namespace rthpva
