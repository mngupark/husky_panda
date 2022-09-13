//
// Created by giuseppe on 01.03.21.
//

#include "husky_panda_rbdl/ros_conversions.h"

namespace husky_panda_rbdl {

void to_msg(const Pose& pose, geometry_msgs::Pose& pose_ros) {
  pose_ros.position.x = pose.translation.x();
  pose_ros.position.y = pose.translation.y();
  pose_ros.position.z = pose.translation.z();
  pose_ros.orientation.x = pose.rotation.x();
  pose_ros.orientation.y = pose.rotation.y();
  pose_ros.orientation.z = pose.rotation.z();
  pose_ros.orientation.w = pose.rotation.w();
}
}  // namespace husky_panda_rbdl