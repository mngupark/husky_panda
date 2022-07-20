#include "husky_panda_hw_interface.h"

namespace husky_panda_control
{
HuskyPandaHW::HuskyPandaHW(const ros::NodeHandle &nh, urdf::Model * urdf_model)
    :   nh_(nh), urdf_model_(urdf_model)
{
    
}
}