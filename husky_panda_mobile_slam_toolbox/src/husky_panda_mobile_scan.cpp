#include <iostream>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/LinearMath/Quaternion.h"
#include <unistd.h>

// scan tf 5.1cm from mount link
// tf sick: laser_mount_link -> laser
// tf hokuyo: hokuyo_laser_mount -> hokuyo laser
int main(int argc, char ** argv) {
    ros::init(argc, argv, "husky_panda_mobile_scan_node");
    ros::NodeHandle nh("~");

    std::string front_scan, rear_scan;
    nh.param<std::string>("front_scan", front_scan, "/sick_scan");
    nh.param<std::string>("rear_scan", rear_scan, "/hokuyo_scan");
    
    tf::TransformListener front_scan_listener, rear_scan_listener;

    while(ros::ok()) {
        tf::StampedTransform front_scan_transform, rear_scan_transform;
        
    }
    return 0;
}