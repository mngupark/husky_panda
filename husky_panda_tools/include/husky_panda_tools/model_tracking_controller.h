//
// Created by giuseppe on 24.04.21.
//

#pragma once
#include <husky_panda_control/core/solver.h>
#include "husky_panda_tools/control_gui.hpp"

//  This class implements a model tracking controller.
//  The same dynamics used for rollouts is used as simulated (controlled)
//  system (no model mismatch)

namespace husky_panda_tools {

class ModelTrackingController {
 public:
  ModelTrackingController() = default;

  void init(husky_panda_control::dynamics_ptr dynamics, husky_panda_control::cost_ptr cost,
            husky_panda_control::policy_ptr policy,
            const husky_panda_control::observation_t& x0, const double& t0,
            const husky_panda_control::config_t& config);
  void get_state(husky_panda_control::observation_t& x) const;
  void set_initial_state(const husky_panda_control::observation_t& x0, const double& t0);
  void set_reference(husky_panda_control::reference_trajectory_t& ref);
  void step();

 public:
  bool initialized_ = false;

  double t_;
  husky_panda_control::input_t u_;
  husky_panda_control::observation_t x_;

  husky_panda_control::dynamics_ptr model_;
  husky_panda_control::solver_ptr solver_;

  husky_panda_tools::ControlGui gui_;
};
}  // namespace husky_panda_tools
