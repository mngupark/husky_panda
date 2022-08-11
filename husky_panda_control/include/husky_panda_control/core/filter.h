//
// Created by giuseppe on 13.09.21.
//

#pragma once
#include "husky_panda_control/core/typedefs.h"

namespace husky_panda_control {

/// Generic interface class for filtering the input
class Filter {
 public:
  Filter() = default;
  ~Filter() = default;

  virtual void reset(const husky_panda_control::observation_t& x, const double t) = 0;
  virtual void apply(const husky_panda_control::observation_t& x, husky_panda_control::input_t& u,
                     const double t) = 0;
};

}  // namespace husky_panda_control
