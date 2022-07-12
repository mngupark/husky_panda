#include "husky_panda_map_conversion.h"

HuksyPandaMapConverter::HuksyPandaMapConverter()
{
    char * home_path = std::getenv("HOME");
    file_name_ = std::string(home_path) + "/door_env.cfg";
    initialize_map_configure(file_name_.c_str());
    ros::NodeHandle nh("~");
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &HuksyPandaMapConverter::map_callback, this);

    ros::spin();
}

HuksyPandaMapConverter::~HuksyPandaMapConverter()
{
}

void HuksyPandaMapConverter::map_callback(const nav_msgs::OccupancyGrid::ConstPtr & map)
{
    generate_map(map, file_name_.c_str());
    return;
}

void HuksyPandaMapConverter::initialize_map_configure(const char * file_name)
{
    ROS_INFO("file name : %s", file_name);
    map_fd_ = open(file_name, O_CREAT | O_RDWR | O_TRUNC, 0644);
    char init_configuration[] = "discretization(cells): 119 119\n"
    "cellsize(meters): 0.025\n"
    "nominalvel(mpersecs): 0.4\n"
    "timetoturn45degsinplace(secs): 5.0\n"
    "start(meters,rads): 1.5 2.0 0 0\n"
    "end(meters,rads): 1.7 1.7 0 1\n"
    "environment:\n";
    write(map_fd_, init_configuration, strlen(init_configuration));
    close(map_fd_);
    return;
}

void HuksyPandaMapConverter::generate_map(const nav_msgs::OccupancyGrid::ConstPtr & map, const char * file_name)
{
    map_fd_ = open(file_name, O_APPEND | O_WRONLY, 0644);
    std::cout << "width: " << map->info.width << "\theight: " << map->info.height << '\n';
    std::cout << "origin: " << map->info.origin.position.x << ", " << map->info.origin.position.y << ", " << map->info.origin.position.z << '\n';
    std::cout << "resolution: " << map->info.resolution << '\n';
    char buf[map->info.width * map->info.height];
    char buf_ptr[256];
    for (auto i = 0; i < map->info.width; i++)
    {
        for (auto j = 0; j < map->info.height; j++)
        {

            int num = (unsigned char)map->data[i * map->info.width + j];
            sprintf(buf_ptr, "%3d ", num);
            write(map_fd_, buf_ptr, strlen(buf_ptr));
        }
        strcpy(buf_ptr, "\n");
        write(map_fd_, buf_ptr, strlen(buf_ptr));
    }
    close(map_fd_);
    ros::shutdown(); // create an environment file only once
    return;
}