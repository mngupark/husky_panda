#pragma once

#include "ros/ros.h"
#include "urdf/model.h"

#include "hardware_interface/robot_hw.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "controller_manager/controller_manager.h"

namespace HuskyPandaWholeBodyController
{
    class HuskyPandaHW : public hardware_interface::RobotHW
    {
    private:
        ros::NodeHandle nh_;

        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;
        hardware_interface::EffortJointInterface effort_state_interface_;

        std::vector<std::string> joint_name_;
        std::size_t num_joints_;
        urdf::Model * urdf_model_;

        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;

        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocity_command_;
        std::vector<double> joint_effort_command_;

    public:
        HuskyPandaHW() = delete;
        HuskyPandaHW(const ros::NodeHandle & nh, urdf::Model * urdf_model = NULL);

        void read(const ros::Time & , const ros::Duration & period) override;
        void update();
        void write(const ros::Time & , const ros::Duration & period) override;

        ~HuskyPandaHW();
    };
} // namespace HuskyPandaWholeBodyController