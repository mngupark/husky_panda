#include <iostream>
#include <memory>
// ROS
#include "ros/ros.h"

// tf
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf_conversions/tf_eigen.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// messages
#include "nav_msgs/Path.h"


class HuskyPandaPathFollower
{
public:
    HuskyPandaPathFollower();
    ~HuskyPandaPathFollower();
private:
    // ROS node handle
    ros::NodeHandle nh_;

    // ROS publisher
    ros::Publisher path_pub_;

    // ROS parameters
    std::string map_frame_id_, odom_frame_id_, base_frame_id_; // tf frame id
    std::string publish_topic_name_;
    
    // ROS messages
    nav_msgs::Path path_msg_;
    geometry_msgs::PoseStamped path_pose_msg_;

    // tf
    tf::TransformListener map_to_odom_listener_, odom_to_base_listener_;
    tf::TransformBroadcaster map_to_base_broadcaster_;
    tf::StampedTransform map_to_odom_transform_, odom_to_base_transform_, map_to_base_transform_;

    // Eigen
    Eigen::Matrix4d map_to_odom_matrix, odom_to_base_matrix, map_to_base_matrix;
    Eigen::Quaterniond map_to_odom_quaternion_, odom_to_base_quaternion_;

private:
    void loop();
    void listenTf();
    void generatePathMsg();
    void publishPath();
};

HuskyPandaPathFollower::HuskyPandaPathFollower()
{
    ros::NodeHandle nh("~");

    nh.param<std::string>("map_frame", map_frame_id_, "map");
    nh.param<std::string>("odom_frame", odom_frame_id_, "odom");
    nh.param<std::string>("base_frame", base_frame_id_, "base_footprint");
    nh.param<std::string>("publish_topic_name", publish_topic_name_, "/path");

    path_pub_ = nh_.advertise<nav_msgs::Path>(publish_topic_name_, 1);

    map_to_base_transform_.frame_id_ = map_frame_id_;
    map_to_base_transform_.child_frame_id_ = "base";

    map_to_odom_matrix.setZero();
    odom_to_base_matrix.setZero();
    map_to_odom_matrix(3, 3) = 1.0;
    odom_to_base_matrix(3, 3) = 1.0;

    loop();
}

HuskyPandaPathFollower::~HuskyPandaPathFollower()
{
}

void HuskyPandaPathFollower::loop()
{
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while(ros::ok())
    {
        listenTf();
        generatePathMsg();
        publishPath();
    }
    spinner.stop();
    ros::shutdown();
}

void HuskyPandaPathFollower::listenTf()
{
    try
    {
        // Verify that TF knows how to transform from the received map frame to the destination base frame
        map_to_odom_listener_.waitForTransform(map_frame_id_, odom_frame_id_, ros::Time(0), ros::Duration(1));
        odom_to_base_listener_.waitForTransform(odom_frame_id_, base_frame_id_, ros::Time(0), ros::Duration(1));

        map_to_odom_listener_.lookupTransform(map_frame_id_, odom_frame_id_,
                                                ros::Time(0), map_to_odom_transform_);
        odom_to_base_listener_.lookupTransform(odom_frame_id_, base_frame_id_,
                                            ros::Time(0), odom_to_base_transform_);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    Eigen::Vector3d temp_vector;
    tf::vectorTFToEigen(map_to_odom_transform_.getOrigin(), temp_vector);
    map_to_odom_matrix.block<3, 1>(0, 3) = temp_vector;
    tf::vectorTFToEigen(odom_to_base_transform_.getOrigin(), temp_vector);
    odom_to_base_matrix.block<3, 1>(0, 3) = temp_vector;

    tf::quaternionTFToEigen(map_to_odom_transform_.getRotation(), map_to_odom_quaternion_);
    map_to_odom_matrix.block<3, 3>(0, 0) = map_to_odom_quaternion_.toRotationMatrix();//.normalized();
    tf::quaternionTFToEigen(odom_to_base_transform_.getRotation(), odom_to_base_quaternion_);
    odom_to_base_matrix.block<3, 3>(0, 0) = odom_to_base_quaternion_.toRotationMatrix();//.normalized();
    
    map_to_base_matrix = map_to_odom_matrix * odom_to_base_matrix;
    // std::cout << map_to_base_matrix << '\n';
    // tf::quaternionTFToMsg((map_to_odom_transform_.getRotation() * odom_to_base_transform_.getRotation()).normalized(), path_pose_msg_.pose.orientation);
    tf::Vector3 tf_vector;
    tf::vectorEigenToTF(map_to_base_matrix.block<3, 1>(0, 3), tf_vector);
    
    path_pose_msg_.pose.position.x = map_to_base_matrix(0, 3);//map_to_odom_transform_.getOrigin().x() + odom_to_base_transform_.getOrigin().x();
    path_pose_msg_.pose.position.y = map_to_base_matrix(1, 3);//map_to_odom_transform_.getOrigin().y() + odom_to_base_transform_.getOrigin().y();
    path_pose_msg_.pose.position.z = map_to_base_matrix(2, 3);//map_to_odom_transform_.getOrigin().z() + odom_to_base_transform_.getOrigin().z();
    Eigen::Quaterniond temp_quaternion(map_to_base_matrix.block<3, 3>(0, 0));
    path_pose_msg_.pose.orientation.x = temp_quaternion.x();
    path_pose_msg_.pose.orientation.y = temp_quaternion.y();
    path_pose_msg_.pose.orientation.z = temp_quaternion.z();
    path_pose_msg_.pose.orientation.w = temp_quaternion.w();

    tf::Quaternion tf_quaternion;
    tf::quaternionEigenToTF(temp_quaternion, tf_quaternion);
    map_to_base_transform_.setOrigin(tf_vector);
    map_to_base_transform_.setRotation(tf_quaternion);
    map_to_base_transform_.stamp_ = ros::Time::now();
}

void HuskyPandaPathFollower::generatePathMsg()
{
    path_msg_.header.frame_id = map_frame_id_;
    path_msg_.header.seq++;
    path_msg_.header.stamp = ros::Time::now();
    path_pose_msg_.header = path_msg_.header;
    path_msg_.poses.push_back(path_pose_msg_);
}

void HuskyPandaPathFollower::publishPath()
{
    map_to_base_broadcaster_.sendTransform(map_to_base_transform_);
    path_pub_.publish(path_msg_);
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "husky_panda_path_node");
    auto node = std::make_shared<HuskyPandaPathFollower>();
    return 0;
}