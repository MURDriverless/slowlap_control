cmake_minimum_required(VERSION 3.0.2)
project(slowlap_planner)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mur_common
  nav_msgs
  roscpp
  std_msgs
)

set (CMAKE_CXX_FLAGS_DEBUG "-g")
set (CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Boost COMPONENTS math)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
catkin_package(CATKIN_DEPENDS)


add_executable(slowlap_planner src/main.cpp)

add_library(node src/node.cpp)
add_library(path_planner src/path_planner.cpp)


target_link_libraries(slowlap_planner ${catkin_LIBRARIES} node path_planner) 

