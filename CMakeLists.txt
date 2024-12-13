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
  message_generation
)


find_package(PCL REQUIRED)
find_package(Boost 1.54 REQUIRED)

add_message_files(
  FILES
  node.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES travel
 CATKIN_DEPENDS roscpp std_msgs message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_main src/main.cpp)
target_link_libraries(${PROJECT_NAME}_main
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


