/*!
 * @file     pendulum_cart_cost.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <husky_panda_rbdl/model.h>

#include <husky_panda_control/core/cost.h>
#include <ros/ros.h>

#include <ros/package.h>
#include "husky_panda_manipulation/dimensions.h"

namespace husky_panda_control {

class HuskyPandaMobileCost : public husky_panda_control::Cost {
 public:
  HuskyPandaMobileCost(const std::string& robot_description,
                  const double linear_weight, const double angular_weight,
                  const double obstacle_radius, bool joint_limits = false, bool holonomic = false);
  ~HuskyPandaMobileCost() = default;

 private:
  bool joint_limits_;
  double linear_weight_;
  double angular_weight_;
  double obstacle_radius_;

  std::string robot_description_;
  husky_panda_rbdl::RobotModel robot_model_;
  std::string tracked_frame_ = "panda_grasp";

  Eigen::Matrix<double, 3, 3> Q_linear_;
  Eigen::Matrix<double, 3, 3> Q_angular_;

  double Q_obst_ = 100000;
  double Q_reach_ = 100000;
  bool obstacle_set_ = false;
  Eigen::Vector3d distance_vector_;
  bool holonomic_;

  Eigen::Matrix<double, 10, 1> joint_limits_lower_;
  Eigen::Matrix<double, 10, 1> joint_limits_upper_;

 public:
  husky_panda_control::cost_ptr create() override;
  husky_panda_control::cost_ptr clone() const override;

  void set_linear_weight(const double k);
  void set_angular_weight(const double k);
  void set_obstacle_radius(const double r);
  husky_panda_rbdl::Pose get_current_pose(const Eigen::VectorXd& x);
  husky_panda_control::cost_t compute_cost(const husky_panda_control::observation_t& x,
                            const husky_panda_control::input_t& u,
                            const husky_panda_control::reference_t& ref,
                            const double t) override;
};
}  // namespace panda_mobile
