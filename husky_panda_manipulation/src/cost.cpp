/*!
 * @file     pendulum_cart_cost.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "husky_panda_manipulation/cost.h"

#include <ros/ros.h>
#include <cmath>

#include <ros/package.h>

#define PANDA_UPPER_LIMITS \
  2.0, 2.0, 6.28, 2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973, 0.04, 0.04
#define PANDA_LOWER_LIMITS                                                 \
  -2.0, -2.0, -6.28, -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, \
      -2.8973, 0.0, 0.0

namespace husky_panda_control {

HuskyPandaMobileCost::HuskyPandaMobileCost(const std::string& robot_description,
                                 const double linear_weight,
                                 const double angular_weight,
                                 const double obstacle_radius,
                                 bool joint_limits,
                                 bool holonomic)
    : robot_description_(robot_description),
      linear_weight_(linear_weight),
      angular_weight_(angular_weight),
      obstacle_radius_(obstacle_radius),
      joint_limits_(joint_limits),
      holonomic_(holonomic) {
  robot_model_.init_from_xml(robot_description);

  Q_linear_ = Eigen::Matrix3d::Identity() * linear_weight;
  Q_angular_ = Eigen::Matrix3d::Identity() * angular_weight;

  distance_vector_.setZero();

  // // TODO remove hard coded joint limits
  // joint_limits_lower_ << PANDA_LOWER_LIMITS;
  // joint_limits_upper_ << PANDA_UPPER_LIMITS;
}

husky_panda_control::cost_ptr HuskyPandaMobileCost::create() {
  return std::make_shared<HuskyPandaMobileCost>(robot_description_, linear_weight_,
                                           angular_weight_, obstacle_radius_,
                                           joint_limits_, holonomic_);
}

husky_panda_control::cost_ptr HuskyPandaMobileCost::clone() const {
  return std::make_shared<HuskyPandaMobileCost>(*this);
}

void HuskyPandaMobileCost::set_linear_weight(const double k) { Q_linear_ *= k; }

void HuskyPandaMobileCost::set_angular_weight(const double k) { Q_angular_ *= k; }

void HuskyPandaMobileCost::set_obstacle_radius(const double r) {
  obstacle_radius_ = r;
}

husky_panda_rbdl::Pose HuskyPandaMobileCost::get_current_pose(
    const Eigen::VectorXd& x) {
  return robot_model_.get_pose(tracked_frame_, x);
}

husky_panda_control::cost_t HuskyPandaMobileCost::compute_cost(const husky_panda_control::observation_t& x,
                                           const husky_panda_control::input_t& u,
                                           const husky_panda_control::reference_t& ref,
                                           const double t) {
  husky_panda_control::cost_t cost;

  // update model
  robot_model_.update_state(x);

  // target reaching cost
  Eigen::Vector3d ref_t = ref.head<3>();
  Eigen::Quaterniond ref_q(ref.segment<4>(3));
  Eigen::Matrix<double, 6, 1> error;

  husky_panda_rbdl::Pose current_pose = get_current_pose(x);
  husky_panda_rbdl::Pose reference_pose(ref_t, ref_q);

  error = husky_panda_rbdl::diff(current_pose, reference_pose);

  // cost += error.head<3>().transpose() * Q_linear_* error.head<3>();
  // cost += error.tail<3>().transpose() * Q_angular_* error.tail<3>();
  cost += (error.head<3>().transpose() * error.head<3>()).norm() * linear_weight_;
  cost += (error.tail<3>().transpose() * error.tail<3>()).norm() * angular_weight_;

  if (cost < 0.001) cost += 10.0 * u.norm();

  // obstacle avoidance cost
  double obstacle_dist = (current_pose.translation - ref.tail<3>()).norm();
  if (obstacle_dist < obstacle_radius_) cost += Q_obst_;

  // // reach cost
  // if (Q_reach_ > 0) {
  //   double reach = (robot_model_.get_pose(tracked_frame_, x).translation -
  //                   robot_model_.get_pose("panda_link0", x).translation)
  //                      .head<2>()
  //                      .norm();
  //   if (reach > 1.0) {
  //     cost += Q_reach_;
  //   }
  // }

  // arm reach cost
  double reach;
  robot_model_.get_offset("panda_link0", tracked_frame_,
                          distance_vector_, x);
  reach = distance_vector_.head<2>().norm();
  if (reach > 1.0) {
    cost += 100 +
            10 * (std::pow(reach - 1.0, 2)); // reach weight: 100, reach weight slope: 10, max reach: 1.0
  }

  if (distance_vector_.norm() < 0.0) {
    cost += 100 +
            10 * (std::pow(reach - 0.0, 2)); // min dist: 0.0
  }

  // joint limits cost
  if (joint_limits_) {
    for (int i = 0; i < x.size(); i++) {
      if (x(i) < joint_limits_lower_(i))
        cost += 100 + 10 * std::pow(joint_limits_lower_(i) - x(i), 2);
      if (x(i) > joint_limits_upper_(i))
        cost += 100 + 10 * std::pow(x(i) - joint_limits_upper_(i), 2);
    }
  }

  // differential drive cost
  if (holonomic_)
  {
  }
  else
  {
    double w_base = 100.0;
    double w_input = 5.0;
    puts("test");

    husky_panda_rbdl::Pose current_base_pose = robot_model_.get_pose("base_link", x);
    Eigen::Matrix<double, 6, 1> base_error = husky_panda_rbdl::diff(current_base_pose, reference_pose);
    // orientation error
    if (reference_pose.rotation.coeffs().dot(current_base_pose.rotation.coeffs()) < 0.0)
    {
      current_base_pose.rotation.coeffs() << -current_base_pose.rotation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(current_base_pose.rotation.inverse() * reference_pose.rotation);
    base_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    Eigen::Matrix4d T;
    T.setZero();
    T(3, 3) = 1;
    T.block<3, 3>(0, 0) << current_pose.rotation.toRotationMatrix();
    T.block<3, 1>(0, 3) << current_pose.translation;
    Eigen::Affine3d transform(T);
    base_error.tail(3) << -transform.linear() * base_error.tail(3);
    cost += (std::pow(base_error(0), 2) * w_base + std::pow(base_error(1), 2) * w_base + std::pow(base_error(5), 2) * w_input);
    cost += std::pow(u(0), 2) * w_base + std::pow(u(1), 2) * w_input;
  }
  return cost;
}

}  // namespace panda_mobile