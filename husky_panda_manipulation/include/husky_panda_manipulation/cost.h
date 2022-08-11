/*!
 * @file     pendulum_cart_cost.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <math.h>
#include <husky_panda_control/core/cost.h>
#include <mppi_pinocchio/model.h>
#include <ros/ros.h>

#include <ros/package.h>
#include "husky_panda_manipulation/params/cost_params.h"

namespace husky_panda_control {

class PandaCost : public husky_panda_control::Cost {
 public:
  PandaCost() : PandaCost(CostParams()){};
  PandaCost(const CostParams& param);
  ~PandaCost() = default;

  // debug only
  inline const mppi_pinocchio::RobotModel& robot() const {
    return robot_model_;
  }
  inline const mppi_pinocchio::RobotModel& object() const {
    return object_model_;
  }

 private:
  CostParams params_;

  mppi_pinocchio::RobotModel robot_model_;
  mppi_pinocchio::RobotModel object_model_;

  int frame_id_;
  int arm_base_frame_id_;
  Eigen::Matrix<double, 6, 1> error_;
  Eigen::Vector3d distance_vector_;
  Eigen::Vector3d collision_vector_;

 public:
  husky_panda_control::cost_ptr create() override {
    return std::make_shared<PandaCost>(params_);
  }
  husky_panda_control::cost_ptr clone() const override {
    return std::make_shared<PandaCost>(*this);
  }

  void set_linear_weight(const double k) { params_.Qt = k; }
  
  void set_angular_weight(const double k) { params_.Qr = k; }
  
  void set_obstacle_radius(const double r) { params_.ro = r; }

  husky_panda_control::cost_t compute_cost(const husky_panda_control::observation_t& x,
                            const husky_panda_control::input_t& u,
                            const husky_panda_control::reference_t& ref,
                            const double t) override;
};
}  // namespace husky_panda_control
