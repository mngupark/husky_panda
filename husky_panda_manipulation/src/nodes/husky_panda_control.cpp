/*!
 * @file     panda_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */

#include "husky_panda_manipulation/controller_interface.h"
#include "husky_panda_manipulation/dynamics_ros.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <chrono>

using namespace husky_panda_control;

int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "husky_panda_control");
  ros::NodeHandle nh("~");

  auto sequential = nh.param<bool>("sequential", false);
  auto max_sim_time = nh.param<double>("max_sim_time", 0.0);
  auto holonomic = nh.param<bool>("holonomic", false);

  std::string robot_description = nh.param<std::string>("robot_description_dynamics", "");
  std::string obstacle_description = nh.param<std::string>("/obstacle_description", "");
  auto simulation = std::make_shared<HuskyPandaMobileDynamics>(nh, robot_description, obstacle_description, holonomic);
  auto controller = std::make_shared<HuskyPandaMobileControllerInterface>(nh, holonomic);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(HuskyPandaMobileDim::STATE_DIMENSION);
  auto initial_configuration =
      nh.param<std::vector<double>>("initial_configuration", {});
  for (size_t i = 0; i < initial_configuration.size(); i++)
    x(i) = initial_configuration[i];
  // simulation->reset(x, 0.0);

  husky_panda_control::input_t u;
  u = simulation->get_zero_input(x);

  bool static_optimization = nh.param<bool>("static_optimization", false);
  double sim_dt = nh.param<double>("sim_dt", 0.01);

  double sim_time = 0.0;

  // init the controller
  bool ok = controller->init();
  if (!ok) {
    throw std::runtime_error("Failed to initialize the controller!");
  }

  // set the very first observation
  controller->set_observation(x, sim_time);

  if (!sequential) controller->start();
  raisim::RaisimServer server(simulation->get_world());
  server.launchServer();
  server.focusOn(simulation->get_robot());
  while (ros::ok()) {
    auto start = std::chrono::steady_clock::now();
    controller->set_observation(x, sim_time);
    controller->get_input(x, u, sim_time);

    if (sequential) {
      controller->update_reference();
      controller->publish_ros_default();
      controller->publish_ros();
      controller->update_policy();
    }

    if (!static_optimization) {
      x = simulation->step(u, sim_dt);
      sim_time += sim_dt;
    }

    auto end = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count() /
        1000.0;
    if (sim_dt - elapsed > 0) ros::Duration(sim_dt - elapsed).sleep();

    if (max_sim_time > 0 && sim_time > max_sim_time) {
      ROS_INFO_STREAM("Reached maximum sim time: " << max_sim_time
                                                   << "s. Exiting.");
      break;
    }

    simulation->publish_ros();
    ros::spinOnce();
  }

  server.killServer();
  ros::shutdown();

  return 0;
}