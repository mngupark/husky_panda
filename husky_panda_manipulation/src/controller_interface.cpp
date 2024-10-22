/*!
 * @file     controller_interface.cpp
 * @author   Giuseppe Rizzi
 * @date     25.09.2020
 * @version  1.0
 * @brief    description
 */

#include "husky_panda_manipulation/controller_interface.h"
#include "husky_panda_manipulation/cost.h"
#include "husky_panda_manipulation/dynamics.h"

#include <husky_panda_control/policies/gaussian_policy.h>
#include <husky_panda_control/policies/spline_policy.h>
#include <husky_panda_rbdl/ros_conversions.h>
#include "husky_panda_ros/ros_params.h"
#include <ros/package.h>

using namespace husky_panda_control;

bool HuskyPandaMobileControllerInterface::init_ros() {
  optimal_trajectory_publisher_ =
      nh_.advertise<nav_msgs::Path>("/optimal_trajectory", 10);
  optimal_base_trajectory_publisher_ =
      nh_.advertise<nav_msgs::Path>("/optimal_base_trajectory", 10);
  obstacle_marker_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("/obstacle_marker", 10);

  obstacle_subscriber_ =
      nh_.subscribe("/obstacle", 10,
                    &HuskyPandaMobileControllerInterface::obstacle_callback, this);
  ee_pose_desired_subscriber_ = nh_.subscribe(
      "/end_effector_pose_desired", 10,
      &HuskyPandaMobileControllerInterface::ee_pose_desired_callback, this);

  for (size_t i = 0; i < STATE_DIMENSION; i++)
  {
    dynamcis_model_pose_publisher_[i] =
      nh_.advertise<geometry_msgs::PoseStamped>("/dynamics_model_" + std::to_string(i), 10);
    dynamics_model_pose_[i].header.frame_id = "world";
  }

  if (!husky_panda_ros::getNonNegative(nh_, "obstacle_radius", obstacle_radius_))
    return false;

  obstacle_marker_.header.frame_id = "world";
  obstacle_marker_.type = visualization_msgs::Marker::SPHERE;
  obstacle_marker_.color.r = 1.0;
  obstacle_marker_.color.g = 0.0;
  obstacle_marker_.color.b = 0.0;
  obstacle_marker_.color.a = 0.4;
  obstacle_marker_.scale.x = 2.0 * 0.01;
  obstacle_marker_.scale.y = 2.0 * 0.01;
  obstacle_marker_.scale.z = 2.0 * 0.01;
  obstacle_marker_.pose.orientation.x = 0.0;
  obstacle_marker_.pose.orientation.y = 0.0;
  obstacle_marker_.pose.orientation.z = 0.0;
  obstacle_marker_.pose.orientation.w = 1.0;
  obstacle_marker_.pose.position.x = 1.0;
  obstacle_marker_.pose.position.y = 0.0;
  obstacle_marker_.pose.position.z = 1.0;

  last_ee_ref_id_ = 0;
  ee_desired_pose_.header.seq = last_ee_ref_id_;

  last_ob_ref_id_ = 0;
  obstacle_pose_.header.seq = last_ob_ref_id_;

  optimal_path_.header.frame_id = "world";
  optimal_base_path_.header.frame_id = "world";
  return true;
}

void HuskyPandaMobileControllerInterface::init_model(
    const std::string& robot_description) {
  robot_model_.init_from_xml(robot_description);
}

bool HuskyPandaMobileControllerInterface::set_controller(
    husky_panda_control::solver_ptr& controller) {
  // Params
  std::string robot_description_planar, robot_description_dynamics, robot_description_model, obstacle_description;
  double linear_weight;
  double angular_weight;
  bool joint_limits;
  std::vector<double> joint_limits_upper, joint_limits_lower;
  bool gaussian_policy;
  bool holonomic;

  bool ok = true;
  ok &= husky_panda_ros::getString(nh_, "/robot_description_planar", robot_description_planar);
  ok &= husky_panda_ros::getString(nh_, "/robot_description_model", robot_description_model);
  ok &= husky_panda_ros::getString(nh_, "robot_description_dynamics", robot_description_dynamics);
  ok &= husky_panda_ros::getString(nh_, "/obstacle_description", obstacle_description);
  ok &= husky_panda_ros::getNonNegative(nh_, "obstacle_radius", obstacle_radius_);
  ok &= husky_panda_ros::getNonNegative(nh_, "linear_weight", linear_weight);
  ok &= husky_panda_ros::getNonNegative(nh_, "angular_weight", angular_weight);
  ok &= husky_panda_ros::getBool(nh_, "joint_limits", joint_limits);
  if (!nh_.getParam("joint_limits_upper", joint_limits_upper) ||
      joint_limits_upper.size() != ARM_GRIPPER_DIMENSION) {
    ROS_ERROR("Failed to get joint_limits_upper parameter");
    return false;
  }
  if (!nh_.getParam("joint_limits_lower", joint_limits_lower) ||
      joint_limits_lower.size() != ARM_GRIPPER_DIMENSION) {
    ROS_ERROR("Failed to get joint_limits_upper parameter");
    return false;
  }
  ok &= husky_panda_ros::getBool(nh_, "gaussian_policy", gaussian_policy);
  ok &= husky_panda_ros::getBool(nh_, "holonomic", holonomic);
  if (!ok) {
    ROS_ERROR("Failed to parse parameters and set controller.");
    return false;
  }

  // -------------------------------
  // config
  // -------------------------------
  std::string config_file;
  if (holonomic)
  {
    config_file =
      ros::package::getPath("husky_panda_manipulation") + "/config/holonomic_params.yaml";
  }
  else
  {
    config_file =
      ros::package::getPath("husky_panda_manipulation") + "/config/nonholonomic_params.yaml";
  }
  if (!config_.init_from_file(config_file)) {
    ROS_ERROR_STREAM("Failed to init solver options from " << config_file);
    return false;
  }

  // -------------------------------
  // internal model
  // -------------------------------
  init_model(robot_description_model);

  // -------------------------------
  // dynamics
  // -------------------------------
  auto dynamics =
      std::make_shared<HuskyPandaRaisimDynamics>(robot_description_dynamics, obstacle_description, holonomic);

  // -------------------------------
  // cost
  // -------------------------------
  auto cost = std::make_shared<HuskyPandaMobileCost>(robot_description_model,
                                                linear_weight, angular_weight,
                                                obstacle_radius_,
                                                joint_limits_upper, joint_limits_lower,
                                                joint_limits, holonomic);

  // -------------------------------
  // policy
  // -------------------------------
  std::shared_ptr<husky_panda_control::Policy> policy;
  if (gaussian_policy) {
    if (holonomic)
    {
      policy = std::make_shared<husky_panda_control::GaussianPolicy>(
        int(HuskyPandaMobileDim::INPUT_DIMENSION + 1), config_);
    }
    else
    {
      policy = std::make_shared<husky_panda_control::GaussianPolicy>(
        int(HuskyPandaMobileDim::INPUT_DIMENSION), config_);
    }
  } else {
    policy = std::make_shared<husky_panda_control::SplinePolicy>(
        int(HuskyPandaMobileDim::INPUT_DIMENSION), config_);
  }

  // -------------------------------
  // controller
  // -------------------------------
  controller = std::make_shared<husky_panda_control::Solver>(dynamics, cost, policy, config_);

  // -------------------------------
  // initialize reference
  // -------------------------------
  ref_.rr.resize(
      1, husky_panda_control::observation_t::Zero(HuskyPandaMobileDim::REFERENCE_DIMENSION));
  ref_.tt.resize(1, 0.0);
  // -------------------------------
  // obstacle marker
  // -------------------------------
  obstacle_marker_.scale.x = 2.0 * obstacle_radius_;
  obstacle_marker_.scale.y = 2.0 * obstacle_radius_;
  obstacle_marker_.scale.z = 2.0 * obstacle_radius_;
  return true;
}

void HuskyPandaMobileControllerInterface::ee_pose_desired_callback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  ee_desired_pose_ = *msg;
  Eigen::VectorXd pr = Eigen::VectorXd::Zero(7);
  pr(0) = msg->pose.position.x;
  pr(1) = msg->pose.position.y;
  pr(2) = msg->pose.position.z;
  pr(3) = msg->pose.orientation.x;
  pr(4) = msg->pose.orientation.y;
  pr(5) = msg->pose.orientation.z;
  pr(6) = msg->pose.orientation.w;
  ref_.rr[0].head<7>() = pr;
}

void HuskyPandaMobileControllerInterface::obstacle_callback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  obstacle_pose_ = *msg;
  ref_.rr[0](7) = obstacle_pose_.pose.position.x;
  ref_.rr[0](8) = obstacle_pose_.pose.position.y;
  ref_.rr[0](9) = obstacle_pose_.pose.position.z;
}

bool HuskyPandaMobileControllerInterface::update_reference() {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  if (last_ee_ref_id_ != ee_desired_pose_.header.seq ||
      (last_ob_ref_id_ != obstacle_pose_.header.seq &&
       ee_desired_pose_.header.seq != 0)) {
    get_controller()->set_reference_trajectory(ref_);
  }
  last_ee_ref_id_ = ee_desired_pose_.header.seq;
  last_ob_ref_id_ = obstacle_pose_.header.seq;
  return true;
}

husky_panda_rbdl::Pose HuskyPandaMobileControllerInterface::get_pose_end_effector(
    const Eigen::VectorXd& x) {
  robot_model_.update_state(x);
  return robot_model_.get_pose("panda_grasp", x);
}

geometry_msgs::PoseStamped
HuskyPandaMobileControllerInterface::get_pose_end_effector_ros(
    const Eigen::VectorXd& x) {
  husky_panda_rbdl::Pose pose = get_pose_end_effector(x);
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.stamp = ros::Time::now();
  pose_ros.header.frame_id = "world";
  husky_panda_rbdl::to_msg(pose, pose_ros.pose);
  return pose_ros;
}

geometry_msgs::PoseStamped HuskyPandaMobileControllerInterface::get_pose_base(
    const husky_panda_control::observation_t& x) {
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.stamp = ros::Time::now();
  pose_ros.header.frame_id = "world";
  pose_ros.pose.position.x = x(0);
  pose_ros.pose.position.y = x(1);
  pose_ros.pose.position.z = 0.0;
  pose_ros.pose.orientation.x = 0.0;
  pose_ros.pose.orientation.y = 0.0;
  pose_ros.pose.orientation.z = std::sin(0.5 * x(2));
  pose_ros.pose.orientation.w = std::cos(0.5 * x(2));
  return pose_ros;
}

void HuskyPandaMobileControllerInterface::publish_ros() {
  if (obstacle_pose_.header.seq != 0) {  // obstacle set at least once
    obstacle_marker_.pose.position = obstacle_pose_.pose.position;
    obstacle_marker_publisher_.publish(obstacle_marker_);
  }

  optimal_path_.header.stamp = ros::Time::now();
  optimal_path_.poses.clear();

  optimal_base_path_.header.stamp = ros::Time::now();
  optimal_base_path_.poses.clear();

  husky_panda_rbdl::Pose pose_temp;
  geometry_msgs::PoseStamped pose_temp_ros;
  pose_temp_ros.header.frame_id = "world";
  get_controller()->get_optimal_rollout(x_opt_, u_opt_);

  for (const auto& x : x_opt_) {
    pose_temp = get_pose_end_effector(x);
    husky_panda_rbdl::to_msg(pose_temp, pose_temp_ros.pose);
    optimal_path_.poses.push_back(pose_temp_ros);
    optimal_base_path_.poses.push_back(get_pose_base(x));
  }

  optimal_trajectory_publisher_.publish(optimal_path_);
  optimal_base_trajectory_publisher_.publish(optimal_base_path_);

  for (size_t j = 0; j < STATE_DIMENSION; j++)
  {
    // rbdl
    // pose_temp = robot_model_.get_pose(j, get_controller()->dynamics_->get_state());
    pose_temp = robot_model_.get_pose(robot_model_.frame_id_[j], get_controller()->dynamics_->get_state());
    husky_panda_rbdl::to_msg(pose_temp, dynamics_model_pose_[j].pose);
    dynamics_model_pose_[j].header.stamp = ros::Time::now();
    dynamcis_model_pose_publisher_[j].publish(dynamics_model_pose_[j]);
  }
}