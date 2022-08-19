cmake_minimum_required(VERSION 3.10)
project(husky_panda_manipulation)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED 3.3)
find_package(raisim CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

set(PACKAGE_DEPENDENCIES
        sensor_msgs
        std_msgs
        roscpp
        husky_panda_control
        husky_panda_ros
        husky_panda_rbdl
        mppi_panda_mobile
        roslib
        roscpp
        husky_panda_msgs
        tf2_ros
        # signal_logger
        )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
        DEPENDS raisim)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/dynamics_ros.cpp
        src/dynamics.cpp
        src/cost.cpp
        src/controller_interface.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC raisim::raisim ${catkin_LIBRARIES} yaml-cpp)

add_executable(husky_panda_control src/nodes/husky_panda_control.cpp)
target_link_libraries(husky_panda_control ${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(husky_panda_control ${PROJECT_NAME})

install(DIRECTORY launch resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
