//
// Created by giuseppe on 18.01.21.
//

#pragma once

#include "husky_panda_manipulation/dynamics.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

namespace husky_panda_control {

class HuskyPandaMobileDynamics : public HuskyPandaRaisimDynamics {
 public:
  HuskyPandaMobileDynamics(const ros::NodeHandle& nh,
      const std::string& robot_description, const std::string& obstacle_description);
  ~HuskyPandaMobileDynamics() = default;

 public:
  void reset_to_default();
  void publish_ros();
  void obstacle_callback(const geometry_msgs::PoseStampedConstPtr& msg);
  void reference_callback(const geometry_msgs::PoseStampedConstPtr& msg);

 private:
  ros::NodeHandle nh_;
  ros::Publisher state_publisher_;
  ros::Publisher ee_publisher_;
  ros::Subscriber obstacle_subscriber_;
  ros::Subscriber reference_subscriber_;

  sensor_msgs::JointState joint_state_;
};

}  // namespace husky_panda_control
