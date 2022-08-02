# husky_panda
## Description
- husky_panda_control : A control package for husky_pand which contains the ROS Control package plugins, params, etc.
- husky_panda_description : A description package of robots which are containing Clearpath robotics Husky A200 and Franka Emika robotics Panda.
- husky_panda_slam : A SLAM package for husky_panda which contains the Slam_toolbox, Cartographer, gmapping wrapper
- husky_panda_manipulation : A control of manipulation package for husky_panda

## Dependencies
- libfranka
- franka_ros

## Build
When you build the *husky_panda_control* package, you need to add extra argument with "-DFranka_DIR:PATH=<path/of/the/libfranka/build>.