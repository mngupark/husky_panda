#include "husky_panda_hw_interface.h"

namespace HuskyPandaWholeBodyController
{
HuskyPandaHW::HuskyPandaHW(const ros::NodeHandle &nh, urdf::Model * urdf_model)
    :   nh_(nh), urdf_model_(urdf_model)
{
    
}
}