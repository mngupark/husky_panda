#pragma once
#include <husky_panda_rbdl/model.h>
#include "husky_panda_ros/controller_interface.h"

#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include "husky_panda_manipulation/dimensions.h"

namespace husky_panda_control {

class HuskyPandaMobileControllerInterface : public husky_panda_ros::ControllerRos {
 public:
  explicit HuskyPandaMobileControllerInterface(ros::NodeHandle& nh)
      : ControllerRos(nh){};
  ~HuskyPandaMobileControllerInterface() = default;

  bool init_ros() override;
  void publish_ros() override;
  bool update_reference() override;

  husky_panda_rbdl::Pose get_pose_end_effector(const husky_panda_control::observation_t& x);
  geometry_msgs::PoseStamped get_pose_base(const husky_panda_control::observation_t& x);
  geometry_msgs::PoseStamped get_pose_end_effector_ros(
      const husky_panda_control::observation_t& x);

 private:
  void init_model(const std::string& robot_description);
  bool set_controller(husky_panda_control::solver_ptr& controller) override;

  void ee_pose_desired_callback(const geometry_msgs::PoseStampedConstPtr& msg);
  void obstacle_callback(const geometry_msgs::PoseStampedConstPtr& msg);

 public:
  husky_panda_control::Config config_;

 private:
  husky_panda_control::input_array_t u_opt_;
  husky_panda_control::observation_array_t x_opt_;

  size_t last_ee_ref_id_;
  size_t last_ob_ref_id_;
  std::mutex reference_mutex_;
  husky_panda_control::reference_trajectory_t ref_;

  double obstacle_radius_;
  husky_panda_rbdl::RobotModel robot_model_;

  // ros
  ros::Publisher optimal_trajectory_publisher_;
  ros::Publisher optimal_base_trajectory_publisher_;
  ros::Publisher obstacle_marker_publisher_;
  ros::Publisher pinocchio_base_publisher_, pinocchio_ee_publisher_;

  ros::Subscriber obstacle_subscriber_;
  ros::Subscriber ee_pose_desired_subscriber_;

  nav_msgs::Path optimal_path_;
  nav_msgs::Path optimal_base_path_;
  geometry_msgs::PoseStamped obstacle_pose_;
  geometry_msgs::PoseStamped ee_desired_pose_;
  visualization_msgs::Marker obstacle_marker_;
};

}  // namespace panda_mobile