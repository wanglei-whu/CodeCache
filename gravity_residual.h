#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <Eigen/Dense>

struct ImuAccCostFunction {
  ImuAccCostFunction(const Eigen::Vector3d& imu_acc, double dt)
      : imu_acc_(imu_acc), dt_(dt) {
    // Precompute the scaling factor for the five-point difference
    dt2_ = 12.0 * dt * dt;
    inv_dt2_ = 1.0 / dt2_;
  }

  template <typename T>
  bool operator()(const T* const pose_params[5], T* residuals) const {
    // Convert 6D pose parameters (tx, ty, tz, rx, ry, rz) to SE3
    Sophus::SE3<T> poses[5];
    for (int i = 0; i < 5; ++i) {
      Eigen::Map<const Eigen::Matrix<T, 6, 1>> param(pose_params[i]);
      Eigen::Matrix<T, 3, 1> trans = param.template block<3, 1>(0, 0); // translation
      Eigen::Matrix<T, 3, 1> rot_vec = param.template block<3, 1>(3, 0); // rotation vector
      Sophus::SO3<T> rot = Sophus::SO3<T>::exp(rot_vec); // Convert rotation vector to SO3
      poses[i] = Sophus::SE3<T>(trans, rot);
    }

    // Extract positions
    Eigen::Matrix<T, 3, 1> pos[5];
    for (int i = 0; i < 5; ++i) {
      pos[i] = poses[i].translation();
    }

    // Five-point difference: [-1, 16, -30, 16, -1] for global acceleration
    Eigen::Matrix<T, 3, 1> acc_est = T(-1.0) * pos[0] + T(16.0) * pos[1] +
                                     T(-30.0) * pos[2] + T(16.0) * pos[3] +
                                     T(-1.0) * pos[4];
    acc_est *= T(inv_dt2_);

    // Add gravity (z-axis down in global frame)
    Eigen::Matrix<T, 3, 1> gravity(T(0.0), T(0.0), T(9.80665));
    acc_est += gravity;

    // Transform IMU acceleration from local to global frame using pose2 rotation
    Eigen::Matrix<T, 3, 3> R = poses[2].rotationMatrix();
    Eigen::Matrix<T, 3, 1> imu_acc_global = R * Eigen::Matrix<T, 3, 1>(
        T(imu_acc_[0]), T(imu_acc_[1]), T(imu_acc_[2]));

    // Residual: Global estimated acceleration - Global IMU acceleration
    residuals[0] = acc_est[0] - imu_acc_global[0];
    residuals[1] = acc_est[1] - imu_acc_global[1];
    residuals[2] = acc_est[2] - imu_acc_global[2];

    return true;
  }

  // Analytical Jacobian
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const {
    // Call operator() to compute residuals
    (*this)(parameters, residuals);

    if (!jacobians) return true;

    // Precompute constants
    const double coeff = 1.0 / dt2_;
    const double jac_scale[] = {-1.0 * coeff, 16.0 * coeff, -30.0 * coeff,
                                16.0 * coeff, -1.0 * coeff};

    // Rotation matrix and its inverse from pose2 (center pose)
    Sophus::SE3d pose2 = Sophus::SE3d(Eigen::Map<const Eigen::Matrix<double, 6, 1>>(parameters[2]));
    Eigen::Matrix3d R = pose2.rotationMatrix();
    Eigen::Matrix3d R_transpose = R.transpose();
    Eigen::Matrix3d skew_imu_acc = Sophus::SO3d::hat(imu_acc_);

    for (int i = 0; i < 5; ++i) {
      if (jacobians[i]) {
        Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J(
            jacobians[i]);
        J.setZero(); // Initialize to zero

        // Jacobian w.r.t. translation (first 3 columns)
        J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * jac_scale[i];

        // Jacobian w.r.t. rotation (last 3 columns) for pose2 only affects
        // IMU acceleration transformation
        if (i == 2) {
          // Compute Jacobian of R * imu_acc w.r.t. rotation vector
          // Approximate Jacobian using the Lie algebra mapping
          Eigen::Matrix<double, 3, 3> J_rot = -R_transpose * skew_imu_acc;
          J.block<3, 3>(0, 3) = J_rot;
        }
      }
    }

    return true;
  }

 private:
  const Eigen::Vector3d imu_acc_;
  const double dt_;
  double dt2_;  // 12 * dt * dt
  double inv_dt2_; // 1 / (12 * dt * dt)
};

int main() {
  // Example usage
  Sophus::SE3d poses[5];
  // Initialize poses with some values (e.g., identity for testing)
  for (int i = 0; i < 5; ++i) {
    poses[i].setIdentity();
  }

  Eigen::Vector3d imu_acc(0.0, 0.0, 9.80665); // Example IMU acceleration in local frame
  double dt = 0.1; // Example time step

  // Create problem
  ceres::Problem problem;
  double* pose_params[5];
  for (int i = 0; i < 5; ++i) {
    pose_params[i] = new double[6];
    Eigen::Map<Eigen::Matrix<double, 6, 1>>(pose_params[i]) <<
        poses[i].translation(), Sophus::SO3d(poses[i].unit_quaternion()).log(); // Convert to rotation vector
    problem.AddParameterBlock(pose_params[i], 6);
  }

  // Add cost function
  ceres::CostFunction* cost_function = new ceres::DynamicAutoDiffCostFunction<ImuAccCostFunction, 3>(
      new ImuAccCostFunction(imu_acc, dt));
  for (int i = 0; i < 5; ++i) {
    cost_function->AddParameterBlock(6); // Each pose has 6 parameters
  }
  problem.AddResidualBlock(cost_function, nullptr, pose_params[0], pose_params[1],
                           pose_params[2], pose_params[3], pose_params[4]);

  // Solve
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  // Clean up
  for (int i = 0; i < 5; ++i) {
    delete[] pose_params[i];
  }

  return 0;
}
