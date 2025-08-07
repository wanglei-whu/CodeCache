
#pragma once

#include "common/eigen_types.h"

#include <string>
#include <tuple>
#include <vector>

namespace rthpva {
namespace localization {

class LidarCorrespondenceManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Vector3dTuple2 = std::tuple<Eigen::Vector3d, Eigen::Vector3d>;
  using Vector3dTuple3 =
      std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>;
  using Vector3dTuple4 = std::tuple<Eigen::Vector3d, Eigen::Vector3d,
                                    Eigen::Vector3d, Eigen::Vector3d>;

  LidarCorrespondenceManager() = default;

  void insertMatchedP2Point(const Eigen::Vector3d& p1,
                            const Eigen::Vector3d& p2);

  void insertMatchedP2Line(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                           const Eigen::Vector3d& n);

  void insertMatchedP2Plane(const Eigen::Vector3d& p1,
                            const Eigen::Vector3d& p2,
                            const Eigen::Vector3d& n);

  void insertMatchedP2Plane(const Eigen::Vector3d& p1,
                            const Eigen::Vector3d& p20,
                            const Eigen::Vector3d& p21,
                            const Eigen::Vector3d& p22);

  void insertMatchedP2Grid(const Eigen::Vector3d& mean,
                           const Eigen::Vector3d& eigen_value,
                           const Eigen::Vector3d& eigen_rot,
                           const Eigen::Vector3d& p);

  const std::vector<Vector3dTuple2>& getMatchedP2Point() const;

  const std::vector<Vector3dTuple3>& getMatchedP2Line() const;

  const std::vector<Vector3dTuple3>& getMatchedP2Plane() const;

  const std::vector<Vector3dTuple4>& getMatchedP2Grid() const;

  void saveP2Line(const std::string& file);

  void saveP2Plane(const std::string& file);

  void clear();

 private:
  std::vector<Vector3dTuple2> matched_p2point_;
  Matrix3dVector p2point_covariances_;

  std::vector<Vector3dTuple3> matched_p2line_;
  Matrix3dVector p2line_covariance_;

  std::vector<Vector3dTuple3> matched_p2plane_;
  Matrix3dVector p2plane_covariance_;

  std::vector<Vector3dTuple4> matched_p2grid_;
};

}  // namespace localization
}  // namespace rthpva