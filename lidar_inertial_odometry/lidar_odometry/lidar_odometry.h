#pragma once

#include "common/eigen_types.h"

namespace rthpva {
namespace localization {

class LidarOdometry {
 public:
  struct Parameter {
    double ceil_size = 0.2;
    bool use_normal= false;
  };

  LidarOdometry(const Parameter& parameter);
  ~LidarOdometry() = default;

 private:
};

}  // namespace localization
}  // namespace rthpva