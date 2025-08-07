
#include <fstream>
#include <iostream>

#include "lidar_inertial_odometry/lidar_odometry/lidar_correspondence_manager.h"

namespace rthpva {
namespace localization {

void LidarCorrespondenceManager::insertMatchedP2Point(const Eigen::Vector3d& p1,
                                                      const Eigen::Vector3d& p2) {
  matched_p2point_.emplace_back(p1, p2);
}

void LidarCorrespondenceManager::insertMatchedP2Line(const Eigen::Vector3d& p1,
                                                     const Eigen::Vector3d& p2,
                                                     const Eigen::Vector3d& n) {
  matched_p2line_.emplace_back(p1, p2, n);
}

void LidarCorrespondenceManager::insertMatchedP2Plane(const Eigen::Vector3d& p1,
                                                      const Eigen::Vector3d& p2,
                                                      const Eigen::Vector3d& n) {
  matched_p2plane_.emplace_back(p1, p2, n);
}

void LidarCorrespondenceManager::insertMatchedP2Plane(const Eigen::Vector3d& p1,
                                                      const Eigen::Vector3d& p20,
                                                      const Eigen::Vector3d& p21,
                                                      const Eigen::Vector3d& p22) {
  Eigen::Vector3d n = (p21 - p20).cross(p22 - p20);
  insertMatchedP2Plane(p1, p20, n);
}

void LidarCorrespondenceManager::insertMatchedP2Grid(const Eigen::Vector3d& mean,
                                                     const Eigen::Vector3d& eigen_value,
                                                     const Eigen::Vector3d& eigen_rot,
                                                     const Eigen::Vector3d& p) {
  matched_p2grid_.emplace_back(mean, eigen_value, eigen_rot, p);
}

const std::vector<LidarCorrespondenceManager::Vector3dTuple2>&
LidarCorrespondenceManager::getMatchedP2Point() const {
  return matched_p2point_;
}

const std::vector<LidarCorrespondenceManager::Vector3dTuple3>&
LidarCorrespondenceManager::getMatchedP2Line() const {
  return matched_p2line_;
}

const std::vector<LidarCorrespondenceManager::Vector3dTuple3>&
LidarCorrespondenceManager::getMatchedP2Plane() const {
  return matched_p2plane_;
}

const std::vector<LidarCorrespondenceManager::Vector3dTuple4>&
LidarCorrespondenceManager::getMatchedP2Grid() const {
  return matched_p2grid_;
}

void LidarCorrespondenceManager::clear() {
  matched_p2point_.clear();
  matched_p2line_.clear();
  matched_p2plane_.clear();
  matched_p2grid_.clear();
}

void LidarCorrespondenceManager::saveP2Line(const std::string& file) {
  std::ofstream fout(file);
  if (!fout.is_open()) {
    std::cerr << "LidarCorrespondenceManager: failed to saveP2Line matches!" << std::endl;
    return;
  }

  for (auto& item : matched_p2line_) {
    fout << std::get<0>(item).transpose() << " ";
    fout << std::get<1>(item).transpose() << " ";
    fout << std::get<2>(item).transpose() << " ";

    fout << std::endl;
  }

  fout.close();
}

void LidarCorrespondenceManager::saveP2Plane(const std::string& file) {
  std::ofstream fout(file);
  if (!fout.is_open()) {
    std::cerr << "LidarCorrespondenceManager: failed to saveP2Plane matches!" << std::endl;
    return;
  }

  for (auto& item : matched_p2plane_) {
    fout << std::get<0>(item).transpose() << " ";
    fout << std::get<1>(item).transpose() << " ";
    fout << std::get<2>(item).transpose() << " ";

    fout << std::endl;
  }

  fout.close();
}

}  // namespace localization
}  // namespace rthpva