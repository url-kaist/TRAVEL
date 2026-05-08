cmake_minimum_required(VERSION 3.0.2)
project(travel)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# --- catkin / ROS1 dependencies -------------------------------------------
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

# --- TRAVEL pure-C++ core (consumed in-tree) -------------------------------
# The core lives at ../cpp/travel and is plain CMake (no ROS). Pulling it in
# via add_subdirectory makes the `travel::travel_core` interface library
# available to catkin executables below. This is the same pattern KISS-Matcher
# uses for its ROS wrapper.
if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/travel/CMakeLists.txt)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/travel
                     ${CMAKE_CURRENT_BINARY_DIR}/travel_core)
else()
    message(FATAL_ERROR
        "Missing ../cpp/travel relative to ros/. The ROS wrapper expects the "
        "TRAVEL C++ core to live alongside it. If you only have the ROS layer, "
        "install travel via cmake --install on cpp/travel and remove this "
        "branch in favor of find_package(travel).")
endif()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_main src/main.cpp)
add_dependencies(${PROJECT_NAME}_main ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_main
    travel::travel_core
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Route TRAVEL_LOG_* in the core through ROS_INFO/WARN/ERROR for this build.
target_compile_definitions(${PROJECT_NAME}_main PRIVATE TRAVEL_USE_ROS_LOGGING)

install(TARGETS ${PROJECT_NAME}_main
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
