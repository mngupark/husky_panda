#pragma once

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <numeric>

#include <raisim/World.hpp>
#include <raisim/configure.hpp>
#include "raisim/RaisimServer.hpp"

#include <ros/package.h>
#include "husky_panda_manipulation/dimensions.h"
#include <husky_panda_control/core/dynamics.h>

namespace husky_panda_control {

class HuskyPandaRaisimDynamics : public husky_panda_control::Dynamics {
public:
  HuskyPandaRaisimDynamics(const std::string& robot_description,
                           const std::string& obstacle_description);
  ~HuskyPandaRaisimDynamics() = default;

public:
  size_t get_input_dimension() override {
    return HuskyPandaMobileDim::INPUT_DIMENSION;
  }
  size_t get_state_dimension() override {
    return HuskyPandaMobileDim::STATE_DIMENSION;
  }

  raisim::World* get_world() { return &sim_; }

  raisim::ArticulatedSystem* get_robot() { return husky_panda_; }

  husky_panda_control::dynamics_ptr create() override {
    return std::make_shared<HuskyPandaRaisimDynamics>(robot_description_, obstacle_description_);
  }

  husky_panda_control::dynamics_ptr clone() const override {
    std::cout << "cannot clone, raisim world copy constructor is deleted. "
                 "Returning empty pointer"
              << std::endl;
    return husky_panda_control::dynamics_ptr();
  }
private:
  void initialize_params();
  void initialize_world(const std::string& robot_description, const std::string& obstacle_description);
  void initialize_pd();
  void set_collision();

public:
  void reset(const husky_panda_control::observation_t& x, const double t) override;
  void advance();
  void set_control(const husky_panda_control::input_t& u);
  husky_panda_control::observation_t step(const husky_panda_control::input_t& u, const double dt) override;
  husky_panda_control::input_t get_zero_input(const husky_panda_control::observation_t& x) override;
  void get_end_effector_pose(Eigen::Vector3d& ee_position, Eigen::Quaterniond& ee_orientation);
  void get_base_pose(Eigen::Vector3d& base_position, Eigen::Quaterniond& base_orientation);

  const husky_panda_control::observation_t get_state() const override;

protected:
  husky_panda_control::observation_t x_, sim_x_;
  double pre_yaw_;

  std::string robot_description_, obstacle_description_;

  size_t robot_dof_;
  size_t input_dimension_, sim_input_dimension_;
  size_t state_dimension_, sim_state_dimension_;

  Eigen::VectorXd joint_p_, joint_v_;
  Eigen::VectorXd obstacle_p_, obstacle_v_;
  Eigen::VectorXd cmd_, cmdv_;
  Eigen::VectorXd cmd_obstacle_;
  raisim::Vec<VIRTUAL_BASE_DIMENSION> ref_p_, cmd_ref_;
  Eigen::VectorXd joint_p_gain_, joint_d_gain_;
  
  double dt_;
  double wheel_radius_multiplier_, wheel_separation_multiplier_;
  double wheel_radius_, wheel_separation_;
  Eigen::MatrixXd constrained_matrix_;// constrained matrix
  raisim::Vec<3> gravity_;

  raisim::World sim_;
  raisim::ArticulatedSystem* husky_panda_;
  raisim::ArticulatedSystem* obstacle_;
  raisim::Sphere* ref_;
};
}  // namespace panda_mobile
