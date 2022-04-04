cmake_minimum_required(VERSION 3.0.2)
project(travel)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  pcl_conversions
  pcl_ros
  cv_bridge
  geometry_msgs
  sensor_msgs
  std_msgs
  node_msg
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ground_planar_modeling REQUIRED)
find_package(node_msg REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES travel
 CATKIN_DEPENDS pcl_ros roscpp std_msgs
 DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${node_msg_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/travel_node.cpp
)

add_executable(${PROJECT_NAME}_node src/travel_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)