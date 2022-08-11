//
// Created by giuseppe on 05.02.21.
//

#pragma once

#include <husky_panda_control/core/config.h>
#include <husky_panda_control/core/data.h>
#include <husky_panda_control/core/rollout.h>
#include <husky_panda_control/core/typedefs.h>

#include "husky_panda_ros/Array.h"
#include "husky_panda_ros/Config.h"
#include "husky_panda_ros/Data.h"
#include "husky_panda_ros/Rollout.h"

namespace husky_panda_ros {

void to_msg(const husky_panda_control::config_t& config, Config& config_ros);
void to_msg(const husky_panda_control::Rollout& rollout, Rollout& rollout_ros, const bool input_only=false, const size_t max_length=1000);
void to_msg(const husky_panda_control::data_t& data, Data& data_ros);

}  // namespace husky_panda_ros