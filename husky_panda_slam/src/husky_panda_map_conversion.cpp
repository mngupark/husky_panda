#include "husky_panda_map_conversion.h"

HuksyPandaMapConverter::HuksyPandaMapConverter()
{
    char * home_path = std::getenv("HOME");
    file_name_ = std::string(home_path) + "/door_env.cfg";
    initialize_map_configure(file_name_.c_str());
    ros::NodeHandle nh("~");
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &HuksyPandaMapConverter::map_callback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/door_path", 1);

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
    // ROS_INFO("file name : %s", file_name);
    // map_fd_ = open(file_name, O_CREAT | O_RDWR | O_TRUNC, 0644);
    // char init_configuration[] = "discretization(cells): 119 119\n"
    // "cellsize(meters): 0.025\n"
    // "nominalvel(mpersecs): 0.4\n"
    // "timetoturn45degsinplace(secs): 5.0\n"
    // "start(meters,rads): 1.5 2.0 0 0\n"
    // "end(meters,rads): 1.7 1.7 0 1\n"
    // "environment:\n";
    // write(map_fd_, init_configuration, strlen(init_configuration));
    // close(map_fd_);

    ROS_INFO("file name : %s", file_name);
    map_fd_ = creat(file_name, 0644);
    return;
}

void HuksyPandaMapConverter::generate_map(const nav_msgs::OccupancyGrid::ConstPtr & map, const char * file_name)
{
    map_fd_ = open(file_name, O_APPEND | O_WRONLY, 0644);
    std::cout << "width: " << map->info.width << "\theight: " << map->info.height << '\n';
    std::cout << "origin: " << map->info.origin.position.x << ", " << map->info.origin.position.y << ", " << map->info.origin.position.z << '\n';
    std::cout << "resolution: " << map->info.resolution << '\n';
    std::cout << "size: " << map->data.size() <<'\n';
    // the origin of map = the origin of the robot calculated from lower-left pose of the map
    float yaw = atan2(2.0 * 
        (map->info.origin.orientation.y * map->info.origin.orientation.z + map->info.origin.orientation.w * map->info.origin.orientation.x),
         map->info.origin.orientation.w * map->info.origin.orientation.w - map->info.origin.orientation.x * map->info.origin.orientation.x
         - map->info.origin.orientation.y * map->info.origin.orientation.y + map->info.origin.orientation.z * map->info.origin.orientation.z);
    char info_buf[256];
    sprintf(info_buf, 
        "discretization(cells): %d %d\n"
        "cellsize(meters): %f\n"
        "nominalvel(mpersecs): 0.4\n"
        "timetoturn45degsinplace(secs): 5.0\n"
        "start(meters,rads): %f %f %f 0\n"
        "end(meters,rads): 1.7 1.7 0 4\n"
        "environment:\n",
        map->info.width, map->info.height,
        map->info.resolution,
        -map->info.origin.position.x, -map->info.origin.position.y, yaw
    );
    write(map_fd_, info_buf, strlen(info_buf));
    // for (int i = 0; i < map->info.height; i++)
    // {
    //     char buf[2048];
    //     // for (int j = map->info.width - 1; j > -1; j--)
    //     for (int j = 0; j < map->info.width; j++)
    //     {
    //         memset(buf, 0, sizeof(buf));
    //         signed int num = (signed char)map->data[i * map->info.width + j];
    //         // if (num != -1)
    //         // {
    //             sprintf(buf, "%3d ", num);
    //             write(map_fd_, buf, strlen(buf));
    //         // }
    //     }
    //     memset(buf, 0, sizeof(buf));
    //     strcpy(buf, "\n");
    //     write(map_fd_, buf, strlen(buf));
    // }
    for (int i = (map->info.width * map->info.height); i > 0; i = i - map->info.height)
    {
        char buf[2048];
        for (int j = 0; j < map->info.width; j++)
        {
            memset(buf, 0, sizeof(buf));
            signed int num = (signed char)map->data[i + j];
            sprintf(buf, "%3d ", num);
            write(map_fd_, buf, strlen(buf));
        }
        memset(buf, 0, sizeof(buf));
        strcpy(buf, "\n");
        write(map_fd_, buf, strlen(buf));
    }
    close(map_fd_);

    // path generation
    // FILE* fp = NULL;
	// char path_line[100];
    //
	// fp = fopen("/home/dyros/temp_catkin/src/mpc_test/data/trajectory/Path.txt", "r");
    //
	// if (fp == NULL) {
	// 	fprintf(stderr, "파일 file.txt를 열 수 없습니다.\n");
	// 	exit(0);
	// }
    //
    // path_msgs_.header.frame_id = "map";
    // path_msgs_.header.stamp = ros::Time::now();
    //
	// while((fgets(path_line, sizeof(path_line), fp)) != NULL)
    // {
    //     char * tok = ",";
    //     char * ptr = strtok(path_line, tok);
    //     double path[3];
    //     path[0] = atof(ptr);
    //     int cnt = 1;
    //     while (cnt < 3)
    //     {
    //         ptr = strtok(NULL, tok);
    //         path[cnt++] = atof(ptr);
    //     }
    //     geometry_msgs::PoseStamped pose;
    //     pose.header.frame_id = "map";
    //     pose.header.stamp = ros::Time::now();
    //     pose.pose.position.x = path[0] + map->info.origin.position.x;
    //     pose.pose.position.y = path[1] + map->info.origin.position.y;
    //     pose.pose.position.z = 0.0;
    //     tf2::Quaternion q;
    //     q.setRPY( 0, 0, path[2] );
    //     tf2::convert(q, pose.pose.orientation);
    //
    //     path_msgs_.poses.push_back(pose);
    // }
    //
    // while (ros::ok())
    // {
    //     path_pub_.publish(path_msgs_);
    // }
    //
	// fclose(fp);
    ros::shutdown(); // create an environment file only once
    return;
}