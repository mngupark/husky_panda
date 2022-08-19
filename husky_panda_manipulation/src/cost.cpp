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
                                 bool joint_limits)
    : robot_description_(robot_description),
      linear_weight_(linear_weight),
      angular_weight_(angular_weight),
      obstacle_radius_(obstacle_radius),
      joint_limits_(joint_limits) {
  robot_model_.init_from_xml(robot_description);

  Q_linear_ = Eigen::Matrix3d::Identity() * linear_weight;
  Q_angular_ = Eigen::Matrix3d::Identity() * angular_weight;

  // // TODO remove hard coded joint limits
  // joint_limits_lower_ << PANDA_LOWER_LIMITS;
  // joint_limits_upper_ << PANDA_UPPER_LIMITS;
}

husky_panda_control::cost_ptr HuskyPandaMobileCost::create() {
  return std::make_shared<HuskyPandaMobileCost>(robot_description_, linear_weight_,
                                           angular_weight_, obstacle_radius_,
                                           joint_limits_);
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
  // printf("ref: %f %f %f\n", ref(0), ref(1), ref(2));
  // husky_panda_rbdl::Pose current_pose = get_current_pose(q);
  husky_panda_rbdl::Pose current_pose = get_current_pose(x);
  husky_panda_rbdl::Pose reference_pose(ref_t, ref_q);
  // std::cout << "pinocchio: " << current_pose.translation.head(2).transpose() << '\n';

  error = husky_panda_rbdl::diff(current_pose, reference_pose);
  // std::cout << "error: " << error.transpose() << '\n';
  cost += error.head<3>().transpose() * Q_linear_ * error.head<3>();
  cost += error.tail<3>().transpose() * Q_angular_ * error.tail<3>();

  // robot_model_.get_error(tracked_frame_, ref_q, ref_t, error);
  //   cost +=
  //       (error.head<3>().transpose() * error.head<3>()).norm() * linear_weight_;
  //   cost +=
  //       (error.tail<3>().transpose() * error.tail<3>()).norm() * angular_weight_;

  // if (cost < 0.001) cost += 10.0 * u.norm();

  // obstacle avoidance cost
  double obstacle_dist = (current_pose.translation - ref.tail<3>()).norm();
  if (obstacle_dist < obstacle_radius_) cost += Q_obst_;

  // reach cost
  if (Q_reach_ > 0) {
    double reach = (robot_model_.get_pose(tracked_frame_, x).translation -
                    robot_model_.get_pose("panda_link0", x).translation)
                       .head<2>()
                       .norm();
    if (reach > 1.0) {
      cost += Q_reach_;
    }
  }

  // // arm reach cost
  // double reach;
  // double min_dist = 0.0;
  // double max_reach = 1.0;
  // double reach_weight = 100;
  // double reach_weight_slope = 10;
  // robot_model_.get_offset("panda_link0", tracked_frame_,
  //                         distance_vector_);
  // reach = distance_vector_.head<2>().norm();
  // if (reach > max_reach) {
  //   cost += reach_weight +
  //           reach_weight_slope * (std::pow(reach - max_reach, 2));
  // }

  // if (distance_vector_.norm() < min_dist) {
  //   cost += reach_weight +
  //           reach_weight_slope * (std::pow(reach - min_dist, 2));
  // }

  // joint limits cost
  if (joint_limits_) {
    for (int i = 0; i < x.size(); i++) {
      if (x(i) < joint_limits_lower_(i))
        cost += 100 + 10 * std::pow(joint_limits_lower_(i) - x(i), 2);
      if (x(i) > joint_limits_upper_(i))
        cost += 100 + 10 * std::pow(x(i) - joint_limits_upper_(i), 2);
    }
  }

  // // differential drive cost
  // double w_x = 1000.0;
  // double w_y = 1.0;
  // double w_yaw = 1000.0;
  // double w_r = 10.0;
  // double w_w = 10.0;
  // husky_panda_rbdl::Pose current_base_pose = robot_model_.get_pose("base_link", x);
  // Eigen::Matrix<double, 6, 1> e = husky_panda_rbdl::diff(current_base_pose, reference_pose);
  // cost += w_x * std::pow(e(0), 4) + w_y * std::pow(e(1), 2) + w_yaw * std::pow(e(5), 4);// + w_r * std::pow(u(0), 4) + w_r * std::pow(u(2), 4);
  return cost;
}

}  // namespace panda_mobile