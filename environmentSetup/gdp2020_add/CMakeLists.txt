cmake_minimum_required(VERSION 3.0.2)
project(gdp2020_add)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  rospy
  sensor_msgs
  tf
  mavros
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


