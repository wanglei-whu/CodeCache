#pragma once

#include "common/eigen_types.h"

namespace rthpva {
namespace common {

template <typename T>
Matrix3<T> SkewSymmetricMatrix(const Vector<T, 3>& vector) {
  Matrix3<T> cross;
  // clang-format off
  cross << 0,         -vector(2),    vector(1),
           vector(2),  0,           -vector(0),
          -vector(1),  vector(0),    0;
  // clang-format on
  return cross;
}

}  // namespace common
}  // namespace rthpva