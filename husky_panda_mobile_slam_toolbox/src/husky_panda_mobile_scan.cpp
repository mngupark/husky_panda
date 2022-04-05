#include <iostream>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/LinearMath/Quaternion.h"
// #include <pthread.h>

// void * get_front_scan(void * arg) {
//     std::cout << "get front!\n";
// }

// void * get_rear_scan(void * arg) {
//     std::cout << "get rear!\n";
// }

sensor_msgs::LaserScan front_scan, rear_scan;

void front_scan_callback(const sensor_msgs::LaserScan::ConstPtr scan) {
    front_scan = *scan.get();
    ROS_INFO("Received: %s", front_scan);
    return;
}

void rear_scan_callback(const sensor_msgs::LaserScan::ConstPtr scan) {
    rear_scan =*scan.get();
    ROS_INFO("Received: %s", rear_scan);
    return;
}

// scan tf 5.1cm from mount link
// tf sick: laser_mount_link -> laser
// tf hokuyo: hokuyo_laser_mount -> hokuyo_laser
int main(int argc, char ** argv) {
    // initialize
    ros::init(argc, argv, "husky_panda_mobile_scan_node");
    ros::NodeHandle nh("~");

    // ROS param
    std::string front_scan, rear_scan, front_scan_mount_link, rear_scan_mount_link, front_scan_link, rear_scan_link;
    nh.param<std::string>("front_scan", front_scan, "/sick_scan");
    nh.param<std::string>("rear_scan", rear_scan, "/hokuyo_scan");
    nh.param<std::string>("front_scan_mount_link", front_scan_mount_link, "laser_mount_link");
    nh.param<std::string>("rear_scan_mount_link", rear_scan_mount_link, "hokuyo_laser_mount");
    nh.param<std::string>("laser", front_scan_link, "laser");
    nh.param<std::string>("hokuyo_laser", rear_scan_link, "hokuyo_scan");
    
    // ROS Publisher & Subscriber
    ros::Publisher laser_scan_publisher = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);
    ros::Subscriber front_scan_subscriber = nh.subscribe<sensor_msgs::LaserScan>(front_scan, 10, front_scan_callback);
    ros::Subscriber rear_scan_subscriber = nh.subscribe<sensor_msgs::LaserScan>(rear_scan, 10, rear_scan_callback);

    // tf
    tf::TransformBroadcaster fake_scan_broadcaster;
    tf::TransformListener front_scan_listener, rear_scan_listener;

    // // pthread
    // pthread_t front_scan_pth, rear_scan_pth;
    // int pth_id[2];
    // pth_id[0] = pthread_create(&front_scan_pth, NULL, get_front_scan, NULL);
    // pth_id[1] = pthread_create(&rear_scan_pth, NULL, get_rear_scan, NULL);
    // pthread_join(front_scan_pth, NULL);
    // pthread_join(rear_scan_pth, NULL);

    while(ros::ok())
    {
        tf::StampedTransform front_scan_transform, rear_scan_transform;
        /// @todo get laser scan from two LiDARs and fusion them into a unique laser scan.
        ///       Broadcast this scan at the middle pose of each scan.
        try
        {
            front_scan_listener.lookupTransform(front_scan_mount_link, front_scan_link,
                                     ros::Time(0), front_scan_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        try
        {
            rear_scan_listener.lookupTransform(rear_scan_mount_link, rear_scan_link,
                                     ros::Time(0), rear_scan_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        tf::StampedTransform fake_scan_transform;
        fake_scan_transform.setOrigin((front_scan_transform.getOrigin() + rear_scan_transform.getOrigin()) / 2.0);
        fake_scan_transform.setRotation(tf::Quaternion::getIdentity());
        fake_scan_broadcaster.sendTransform(fake_scan_transform);



    }
    ros::spin();
    return 0;
}