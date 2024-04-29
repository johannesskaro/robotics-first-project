cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs sensor_msgs dynamic_reconfigure tf)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

## Declare a catkin package
catkin_package()

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gps_to_odom src/gps_to_odom.cpp)
target_link_libraries(gps_to_odom ${catkin_LIBRARIES})

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

add_executable(lidar_visualization src/lidar_visualization.cpp)
target_link_libraries(lidar_visualization ${catkin_LIBRARIES})


