cmake_minimum_required(VERSION 2.8.3)
project(tcpip_comm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tcpip_comm
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(tcpip_client_lib   src/tcpip_client.cpp)
add_executable(circlerun_server src/circlerun_server.cpp)

target_link_libraries(tcpip_client_lib ${catkin_LIBRARIES})
target_link_libraries(circlerun_server ${catkin_LIBRARIES})
