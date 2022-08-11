//
// Created by giuseppe on 05.02.21.
//

#pragma once

#include <Eigen/Core>

#include "husky_panda_control/core/config.h"
#include "husky_panda_control/core/rollout.h"

namespace husky_panda_control {

struct data_t {
  Config config;

  double step_count;
  double stage_cost;
  double reset_time;
  double optimization_time;

  std::vector<double> rollouts_cost;
  std::vector<double> weights;
  std::vector<Rollout> rollouts;
  Rollout optimal_rollout;
};

}  // namespace husky_panda_control
