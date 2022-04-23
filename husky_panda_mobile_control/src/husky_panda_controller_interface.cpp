#include "husky_panda_controller_interface.h"
#include "pluginlib/class_list_macros.hpp"

namespace HuskyPandaWholeBodyController
{
    HuskyPandaControllerInterface::HuskyPandaControllerInterface()
     : command_struct_()
     , wheel_separation_(0.0)
     , wheel_radius_(0.0)
     , wheel_separation_multiplier(0.0)
     , left_wheel_radius_multiplier(0.0)
     , right_wheel_radius_multiplier(0.0)
     , base_frame_id_("base_link")
     , wheel_joints_size_(0)
    {}

    bool HuskyPandaControllerInterface::init(
        hardware_interface::VelocityJointInterface * hw,
        ros::NodeHandle & root_nh,
        ros::NodeHandle & controller_nh)
    {
        
    }
}
