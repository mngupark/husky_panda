#pragma once
#ifndef __HUSKY_PANDA_MAP_CONVERSION_H__
#define __HUSKY_PANDA_MAP_CONVERSION_H__

// C headers
#include <unistd.h>
#include <sys/fcntl.h>

// C++ STL headers
#include <iostream>
#include <stdlib.h>

// ROS standard headers
#include "ros/ros.h"

// ROS messages headers
#include "nav_msgs/OccupancyGrid.h"

class HuksyPandaMapConverter
{
private:
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr & map);
    void initialize_map_configure(const char * file_name);
    void generate_map(const nav_msgs::OccupancyGrid::ConstPtr & map, const char * file_name);
    void loop();

    ros::Subscriber map_sub_;

    int map_fd_;
    bool is_map_generated_;
    std::string file_name_;

public:

    HuksyPandaMapConverter();
    ~HuksyPandaMapConverter();

    ros::NodeHandle nh_;
};


#endif // __HUSKY_PANDA_MAP_CONVERSION_H__