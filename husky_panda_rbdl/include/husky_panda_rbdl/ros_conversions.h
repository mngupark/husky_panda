//
// Created by giuseppe on 01.03.21.
//

#pragma once

// clang-format off
#include "husky_panda_rbdl/model.h"
#include <geometry_msgs/Pose.h>
// clang-format on

namespace husky_panda_rbdl {

void to_msg(const Pose& pose, geometry_msgs::Pose& pose_ros);
}