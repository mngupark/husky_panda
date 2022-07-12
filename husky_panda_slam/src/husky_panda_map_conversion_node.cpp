#include "husky_panda_map_conversion.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "husky_panda_map_conversion_node");
    auto husky_panda_map_converter = boost::make_shared<HuksyPandaMapConverter>();
    return 0;
}