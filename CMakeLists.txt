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

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/travel_node.cpp
)

add_executable(${PROJECT_NAME}_node src/travel_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_kitti_publisher src/kitti_publisher.cpp)
add_dependencies(${PROJECT_NAME}_kitti_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_kitti_publisher
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )