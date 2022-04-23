#include "husky_panda_controller_interface.h"
#include "pluginlib/class_list_macros.hpp"

namespace HuskyPandaWholeBodyController
{
    HuskyPandaControllerInterface::HuskyPandaControllerInterface()
        : command_struct_(), wheel_separation_(0.0), wheel_radius_(0.0), wheel_separation_multiplier(0.0), left_wheel_radius_multiplier(0.0), right_wheel_radius_multiplier(0.0), base_frame_id_("base_link"), wheel_joints_size_(0)
    {
    }

    bool HuskyPandaControllerInterface::init(
        hardware_interface::VelocityJointInterface *hw,
        ros::NodeHandle &root_nh,
        ros::NodeHandle &controller_nh)
    {
        const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        name_ = complete_ns.substr(id + 1);

        std::vector<std::string> left_wheel_names, right_wheel_names;
        if (!getWheelNames(controller_nh, "left_wheel", left_wheel_names) ||
            !getWheelNames(controller_nh, "right_wheel", right_wheel_names))
        {
            return false;
        }

        if (left_wheel_names.size() != right_wheel_names.size())
        {
            ROS_ERROR_STREAM_NAMED(name_,
                                    "left wheels (" << left_wheel_names.size() << ") != " <<
                                    "right wheels (" << right_wheel_names.size() << ").");
            return false;
        }
        else
        {
            wheel_joints_size_ = left_wheel_joints_.size();

            left_wheel_joints_.resize(wheel_joints_size_);
            right_wheel_joints_.resize(wheel_joints_size_);
        }

        // Odometry related:
        double publish_rate;
        controller_nh.param("publish_rate", publish_rate, 50.0);
        ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                                         << publish_rate << "Hz.");
        publish_period_ = ros::Duration(1.0 / publish_rate);

        controller_nh.param("wheel_separation_multiplier", wheel_separation_multiplier_, wheel_separation_multiplier_);
        ROS_INFO_STREAM_NAMED(name_, "Wheel separation will be multiplied by "
                                         << wheel_separation_multiplier_ << ".");

        if (controller_nh.hasParam("wheel_radius_multiplier"))
        {
            double wheel_radius_multiplier;
            controller_nh.getParam("wheel_radius_multiplier", wheel_radius_multiplier);

            left_wheel_radius_multiplier_ = wheel_radius_multiplier;
            right_wheel_radius_multiplier_ = wheel_radius_multiplier;
        }
        else
        {
            controller_nh.param("left_wheel_radius_multiplier", left_wheel_radius_multiplier_, left_wheel_radius_multiplier_);
            controller_nh.param("right_wheel_radius_multiplier", right_wheel_radius_multiplier_, right_wheel_radius_multiplier_);
        }

        ROS_INFO_STREAM_NAMED(name_, "Left wheel radius will be multiplied by "
                                         << left_wheel_radius_multiplier_ << ".");
        ROS_INFO_STREAM_NAMED(name_, "Right wheel radius will be multiplied by "
                                         << right_wheel_radius_multiplier_ << ".");

        int velocity_rolling_window_size = 10;
        controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
        ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of "
                                         << velocity_rolling_window_size << ".");

        odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

        // Twist command related:
        controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
        ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                                         << cmd_vel_timeout_ << "s.");

        controller_nh.param("allow_multiple_cmd_vel_publishers", allow_multiple_cmd_vel_publishers_, allow_multiple_cmd_vel_publishers_);
        ROS_INFO_STREAM_NAMED(name_, "Allow mutiple cmd_vel publishers is "
                                         << (allow_multiple_cmd_vel_publishers_ ? "enabled" : "disabled"));

        controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
        ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

        controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
        ROS_INFO_STREAM_NAMED(name_, "Odometry frame_id set to " << odom_frame_id_);

        controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
        ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_ ? "enabled" : "disabled"));

        // Velocity and acceleration limits:
        controller_nh.param("linear/x/has_velocity_limits", limiter_lin_.has_velocity_limits, limiter_lin_.has_velocity_limits);
        controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
        controller_nh.param("linear/x/has_jerk_limits", limiter_lin_.has_jerk_limits, limiter_lin_.has_jerk_limits);
        controller_nh.param("linear/x/max_velocity", limiter_lin_.max_velocity, limiter_lin_.max_velocity);
        controller_nh.param("linear/x/min_velocity", limiter_lin_.min_velocity, -limiter_lin_.max_velocity);
        controller_nh.param("linear/x/max_acceleration", limiter_lin_.max_acceleration, limiter_lin_.max_acceleration);
        controller_nh.param("linear/x/min_acceleration", limiter_lin_.min_acceleration, -limiter_lin_.max_acceleration);
        controller_nh.param("linear/x/max_jerk", limiter_lin_.max_jerk, limiter_lin_.max_jerk);
        controller_nh.param("linear/x/min_jerk", limiter_lin_.min_jerk, -limiter_lin_.max_jerk);

        controller_nh.param("angular/z/has_velocity_limits", limiter_ang_.has_velocity_limits, limiter_ang_.has_velocity_limits);
        controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
        controller_nh.param("angular/z/has_jerk_limits", limiter_ang_.has_jerk_limits, limiter_ang_.has_jerk_limits);
        controller_nh.param("angular/z/max_velocity", limiter_ang_.max_velocity, limiter_ang_.max_velocity);
        controller_nh.param("angular/z/min_velocity", limiter_ang_.min_velocity, -limiter_ang_.max_velocity);
        controller_nh.param("angular/z/max_acceleration", limiter_ang_.max_acceleration, limiter_ang_.max_acceleration);
        controller_nh.param("angular/z/min_acceleration", limiter_ang_.min_acceleration, -limiter_ang_.max_acceleration);
        controller_nh.param("angular/z/max_jerk", limiter_ang_.max_jerk, limiter_ang_.max_jerk);
        controller_nh.param("angular/z/min_jerk", limiter_ang_.min_jerk, -limiter_ang_.max_jerk);

        // Publish limited velocity:
        controller_nh.param("publish_cmd", publish_cmd_, publish_cmd_);

        // Publish wheel data:
        controller_nh.param("publish_wheel_joint_controller_state", publish_wheel_joint_controller_state_, publish_wheel_joint_controller_state_);

        // If either parameter is not available, we need to look up the value in the URDF
        bool lookup_wheel_separation = !controller_nh.getParam("wheel_separation", wheel_separation_);
        bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);

        if (!setOdomParamsFromUrdf(root_nh,
                                   left_wheel_names[0],
                                   right_wheel_names[0],
                                   lookup_wheel_separation,
                                   lookup_wheel_radius))
        {
            return false;
        }

        // Regardless of how we got the separation and radius, use them
        // to set the odometry parameters
        const double ws = wheel_separation_multiplier_ * wheel_separation_;
        const double lwr = left_wheel_radius_multiplier_ * wheel_radius_;
        const double rwr = right_wheel_radius_multiplier_ * wheel_radius_;
        odometry_.setWheelParams(ws, lwr, rwr);
        ROS_INFO_STREAM_NAMED(name_,
                              "Odometry params : wheel separation " << ws
                                                                    << ", left wheel radius " << lwr
                                                                    << ", right wheel radius " << rwr);

        if (publish_cmd_)
        {
            cmd_vel_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(controller_nh, "cmd_vel_out", 100));
        }

        // Wheel joint controller state:
        if (publish_wheel_joint_controller_state_)
        {
            controller_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>(controller_nh, "wheel_joint_controller_state", 100));

            const size_t num_wheels = wheel_joints_size_ * 2;

            controller_state_pub_->msg_.joint_names.resize(num_wheels);

            controller_state_pub_->msg_.desired.positions.resize(num_wheels);
            controller_state_pub_->msg_.desired.velocities.resize(num_wheels);
            controller_state_pub_->msg_.desired.accelerations.resize(num_wheels);
            controller_state_pub_->msg_.desired.effort.resize(num_wheels);

            controller_state_pub_->msg_.actual.positions.resize(num_wheels);
            controller_state_pub_->msg_.actual.velocities.resize(num_wheels);
            controller_state_pub_->msg_.actual.accelerations.resize(num_wheels);
            controller_state_pub_->msg_.actual.effort.resize(num_wheels);

            controller_state_pub_->msg_.error.positions.resize(num_wheels);
            controller_state_pub_->msg_.error.velocities.resize(num_wheels);
            controller_state_pub_->msg_.error.accelerations.resize(num_wheels);
            controller_state_pub_->msg_.error.effort.resize(num_wheels);

            for (size_t i = 0; i < wheel_joints_size_; ++i)
            {
                controller_state_pub_->msg_.joint_names[i] = left_wheel_names[i];
                controller_state_pub_->msg_.joint_names[i + wheel_joints_size_] = right_wheel_names[i];
            }

            vel_left_previous_.resize(wheel_joints_size_, 0.0);
            vel_right_previous_.resize(wheel_joints_size_, 0.0);
        }

        setOdomPubFields(root_nh, controller_nh);

        // Get the joint object to use in the realtime loop
        for (size_t i = 0; i < wheel_joints_size_; ++i)
        {
            ROS_INFO_STREAM_NAMED(name_,
                                  "Adding left wheel with joint name: " << left_wheel_names[i]
                                                                        << " and right wheel with joint name: " << right_wheel_names[i]);
            left_wheel_joints_[i] = hw->getHandle(left_wheel_names[i]);   // throws on failure
            right_wheel_joints_[i] = hw->getHandle(right_wheel_names[i]); // throws on failure
        }

        sub_command_ = controller_nh.subscribe("cmd_vel", 1, &DiffDriveController::cmdVelCallback, this);

        // Initialize dynamic parameters
        DynamicParams dynamic_params;
        dynamic_params.left_wheel_radius_multiplier = left_wheel_radius_multiplier_;
        dynamic_params.right_wheel_radius_multiplier = right_wheel_radius_multiplier_;
        dynamic_params.wheel_separation_multiplier = wheel_separation_multiplier_;

        dynamic_params.publish_rate = publish_rate;
        dynamic_params.enable_odom_tf = enable_odom_tf_;

        dynamic_params_.writeFromNonRT(dynamic_params);

        // Initialize dynamic_reconfigure server
        DiffDriveControllerConfig config;
        config.left_wheel_radius_multiplier = left_wheel_radius_multiplier_;
        config.right_wheel_radius_multiplier = right_wheel_radius_multiplier_;
        config.wheel_separation_multiplier = wheel_separation_multiplier_;

        config.publish_rate = publish_rate;
        config.enable_odom_tf = enable_odom_tf_;

        dyn_reconf_server_ = std::make_shared<ReconfigureServer>(dyn_reconf_server_mutex_, controller_nh);

        // Update parameters
        dyn_reconf_server_mutex_.lock();
        dyn_reconf_server_->updateConfig(config);
        dyn_reconf_server_mutex_.unlock();

        dyn_reconf_server_->setCallback(
            boost::bind(&DiffDriveController::reconfCallback, this, boost::placeholders::_1, boost::placeholders::_2));

        return true;
    }
}
