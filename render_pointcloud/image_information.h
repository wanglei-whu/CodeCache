#pragma once
#include <vector>

#include "se3.h"

namespace rthpva {

struct ImageInformation {
 public:
  ImageInformation(int image_index, double measure_time,
                   const std::string& image_path, const common::SE3& pose)
      : image_index_(image_index),
        measure_time_(measure_time),
        image_path_(image_path),
        pose_(pose) {}

  int image_index_;
  double measure_time_;
  std::string image_path_;
  common::SE3 pose_;
};

using ImageInformations = std::vector<ImageInformation>;

}  // namespace rthpva

