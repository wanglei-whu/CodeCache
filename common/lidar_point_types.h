// Copyright 2022. All Rights Reserved.
// Author: Lei wang
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rthpva {
namespace common {

// Ref:
// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php#how-to-add-a-new-pointt-type
struct PointXYZIRT {
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint8_t ring;
  uint16_t timestamp_2us;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointXYZIR {
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace common
}  // namespace rthpva

// if define individual point type,we need register the type
POINT_CLOUD_REGISTER_POINT_STRUCT(rthpva::common::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity,
                                                                          intensity)(uint8_t, ring,
                                                                                     ring))
static_assert(sizeof(rthpva::common::PointXYZIR) == 16, "PointXYZIR size is false!");

POINT_CLOUD_REGISTER_POINT_STRUCT(rthpva::common::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity,
                                                                          intensity)(
                                      uint8_t, ring, ring)(uint16_t, timestamp_2us, timestamp_2us))
static_assert(sizeof(rthpva::common::PointXYZIRT) == 16, "PointXYZIRT size is false!");

namespace rthpva {
namespace common {
using PointXYZ = pcl::PointXYZ;
using PointXYZI = pcl::PointXYZI;

using PointCloudXYZ = pcl::PointCloud<PointXYZ>;
using PointCloudXYZI = pcl::PointCloud<PointXYZI>;
using PointCloudXYZIR = pcl::PointCloud<PointXYZIR>;
using PointCloudXYZIRT = pcl::PointCloud<PointXYZIRT>;
}  // namespace common
}  // namespace rthpva
