//
// Created by giuseppe on 18.01.21.
//

#include "husky_panda_manipulation/dynamics_ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

namespace husky_panda_control {

HuskyPandaMobileDynamics::HuskyPandaMobileDynamics(const ros::NodeHandle& nh, 
    const std::string& robot_description, const std::string& obstacle_description)
    : nh_(nh), HuskyPandaRaisimDynamics(robot_description, obstacle_description) {
  state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  ee_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/end_effector", 10);
  obstacle_subscriber_ =
      nh_.subscribe("/obstacle", 10,
                    &HuskyPandaMobileDynamics::obstacle_callback, this);
  reference_subscriber_ =
      nh_.subscribe("/end_effector_pose_desired", 10,
                    &HuskyPandaMobileDynamics::reference_callback, this);
  joint_state_.name = {
      "front_left_wheel", "front_right_wheel",   "rear_left_wheel", "rear_right_wheel",
      "x_base_joint",     "y_base_joint",        "pivot_joint",
      "panda_joint1",     "panda_joint2",        "panda_joint3",
      "panda_joint4",     "panda_joint5",        "panda_joint6",
      "panda_joint7",     "panda_finger_joint1", "panda_finger_joint2"};

  joint_state_.position.resize(joint_state_.name.size());
  joint_state_.velocity.resize(joint_state_.name.size());
  joint_state_.header.frame_id = "world";
}

void HuskyPandaMobileDynamics::reset_to_default() {
  x_.setZero();
  x_.head<VIRTUAL_BASE_ARM_GRIPPER_DIMENSION>() << 0.0, 0.0, 0.2, 0.0, -0.52, 0.0, -1.785,
      0.0, 1.10, 0.69, 0.04, 0.04;
  reset(x_, 0.0);
  ROS_INFO_STREAM("Reset simulation ot default value: " << x_.transpose());
}

void HuskyPandaMobileDynamics::publish_ros() {
  // update robot state visualization
  joint_state_.header.stamp = ros::Time::now();
  for (size_t j = 0; j < BASE_JOINT_DIMENSION; j++) {
    joint_state_.position[j] = joint_p_(GENERALIZED_COORDINATE + j);
    joint_state_.velocity[j] = joint_v_(GENERALIZED_VELOCITY + j);
    // std::cout << "[" << joint_state_.position[j] << "]";
  }
  
  raisim::Vec<3> base_angvel;
  Eigen::Vector3d base_p, base_v;
  base_v.setZero();
  base_v(0) = joint_v_(0);
  base_v(1) = joint_v_(1);
  husky_panda_->getAngularVelocity(0, base_angvel);
  base_v(2) = base_angvel.e()(2);
  
  for (size_t j = 0; j < VIRTUAL_BASE_DIMENSION; j++) {
    joint_state_.position[BASE_JOINT_DIMENSION + j] = x_(j);
    joint_state_.velocity[BASE_JOINT_DIMENSION + j] = base_v(j);
    // std::cout << "[" << joint_state_.position[j] << "]";
  }
  for (size_t j = 0; j < ARM_GRIPPER_DIMENSION; j++) {
    joint_state_.position[BASE_JOINT_DIMENSION + VIRTUAL_BASE_DIMENSION + j] = x_(VIRTUAL_BASE_DIMENSION + j);
    joint_state_.velocity[BASE_JOINT_DIMENSION + VIRTUAL_BASE_DIMENSION + j] = joint_v_(GENERALIZED_VELOCITY + BASE_JOINT_DIMENSION + j);
    // std::cout << "[" << joint_state_.position[BASE_JOINT_DIMENSION + VIRTUAL_BASE_DIMENSION + j] << "]";
  }
  // std::cout << '\n';
  state_publisher_.publish(joint_state_);

  // publish end effector pose
  Eigen::Vector3d ee_position;
  Eigen::Quaterniond ee_orientation;
  get_end_effector_pose(ee_position, ee_orientation);
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.stamp = ros::Time::now();
  pose_ros.header.frame_id = "world";
  pose_ros.pose.position.x = ee_position.x();
  pose_ros.pose.position.y = ee_position.y();
  pose_ros.pose.position.z = ee_position.z();
  pose_ros.pose.orientation.x = ee_orientation.x();
  pose_ros.pose.orientation.y = ee_orientation.y();
  pose_ros.pose.orientation.z = ee_orientation.z();
  pose_ros.pose.orientation.w = ee_orientation.w();
  ee_publisher_.publish(pose_ros);
}

void HuskyPandaMobileDynamics::obstacle_callback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  geometry_msgs::PoseStamped pose = *msg;
  cmd_obstacle_(0) = pose.pose.position.x;
  cmd_obstacle_(1) = pose.pose.position.y;
  cmd_obstacle_(2) = pose.pose.position.z;
  return;
}

void HuskyPandaMobileDynamics::reference_callback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  geometry_msgs::PoseStamped pose = *msg;
  cmd_ref_(0) = pose.pose.position.x;
  cmd_ref_(1) = pose.pose.position.y;
  cmd_ref_(2) = pose.pose.position.z;
  return;
}

}  // namespace husky_panda_manipulation
