/*!
 * @file     expert_base.h
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <Eigen/Dense>
#include <map>

#include "husky_panda_control/core/config.h"
#include "husky_panda_control/core/dynamics.h"
#include "husky_panda_control/core/typedefs.h"
#include "husky_panda_control/utils/gaussian_sampler.h"

namespace husky_panda_control {
class ExpertBase {
 public:
  ExpertBase(char short_name, config_t config, dynamics_ptr dynamics) {
    short_name_ = short_name;
    config_ = config;
    dynamics_ = dynamics;
  };
  ~ExpertBase() = default;

  virtual void update_expert(std::vector<Eigen::VectorXd> mean) = 0;
  virtual Eigen::VectorXd get_sample(size_t step) = 0;

  virtual Eigen::MatrixXd get_sigma_inv(size_t step) = 0;
  char short_name_;

 protected:
  config_t config_;
  dynamics_ptr dynamics_;
};
}  // namespace husky_panda_control
