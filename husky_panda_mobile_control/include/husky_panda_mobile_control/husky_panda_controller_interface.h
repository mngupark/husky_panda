#pragma once

#include "ros/ros.h"

#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "tf/tfMessage.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "geometry_msgs/TwistStamped.h"

namespace HuskyPandaWholeBodyController
{
    class HuskyPandaControllerInterface : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
    {
    private:
        std::string name_;

        ros::Duration publish_period_;
        ros::Time last_state_publish_time_;
        
        std::vector<hardware_interface::JointHandle> left_wheel_joints_;
        std::vector<hardware_interface::JointHandle> right_wheel_joints_;

        double vel_left_desired_previous_;
        double vel_right_desired_previous_;

        struct Commands
        {
            double linear;
            double angular;
            ros::Time stamp;

            Commands() : linear(0.0), angular(0.0), stamp(0.0) {}
        };

        realtime_tools::RealtimeBuffer<Commands> command_;
        Commands command_struct_;
        ros::Subscriber sub_command_;
        
         /// Publish executed commands
        std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> > cmd_vel_pub_;

        /// Controller state publisher
        std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState> > controller_state_pub_;

        double wheel_separation_;
        double wheel_radius_;

        double wheel_separation_multiplier;
        double left_wheel_radius_multiplier;
        double right_wheel_radius_multiplier;

        std::string base_frame_id_;

        size_t wheel_joints_size_;

        void cmdVelCallback(const geometry_msgs::Twist & command);
        bool getWheelNames(ros::NodeHandle & controller_nh,
                            const std::string & wheel_param,
                            std::vector<std::string> wheel_names);
        
    public:
        HuskyPandaControllerInterface(/* args */);
        ~HuskyPandaControllerInterface();

        bool init(hardware_interface::VelocityJointInterface * hw,
                    ros::NodeHandle & root_nh,
                    ros::NodeHandle & controller_nh);

        void update(const ros::Time & time, const ros::Duration & period);
        void starting(const ros::Time & time);
        void stopping(const ros::Time & );
    };
} // namespace HuskyPandaWholeBodyController
