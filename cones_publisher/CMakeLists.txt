cmake_minimum_required(VERSION 3.0.2)
project(cones_publisher)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  std_msgs
  mur_common
)

add_definitions(-std=c++14)
include_directories(src ${catkin_INCLUDE_DIRS})
catkin_package()

add_executable(cones_publisher src/cones_publisher.cpp)

target_link_libraries(cones_publisher ${catkin_LIBRARIES})
