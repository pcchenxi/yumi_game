cmake_minimum_required(VERSION 2.8.3)
project(yumi_control_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  kdl_parser
  kdl_wrapper
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  message_generation

)
find_package(orocos_kdl REQUIRED)

add_service_files(
  FILES
  GameCmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS orocos_kdl system_lib
  CATKIN_DEPENDS  kdl_parser kdl_wrapper moveit_core moveit_ros_planning_interface roscpp rospy std_msgs message_runtime
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

add_executable(kdl_controller_server src/kdl_controller_server.cpp)
target_link_libraries(kdl_controller_server  ${catkin_LIBRARIES} )
