cmake_minimum_required(VERSION 3.16)
project(travel_ros LANGUAGES C CXX)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread")

# Default to C99 for the C-language probes pulled in by find_package(MPI).
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# --- TRAVEL pure-C++ core (consumed in-tree) -------------------------------
# Same pattern as KISS-Matcher's ros2 wrapper. The core is plain CMake (no
# ROS) and lives at ../cpp/travel; we pull it in via add_subdirectory so the
# `travel::travel_core` interface library is available below. The core
# already calls find_package(MPI) + find_package(PCL) for us, so the ROS
# wrapper does not need to repeat that.
if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/travel/CMakeLists.txt)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/travel
                     ${CMAKE_CURRENT_BINARY_DIR}/travel_core)
else()
    message(FATAL_ERROR
        "Missing ../cpp/travel relative to ros/. The ROS 2 wrapper expects "
        "the TRAVEL C++ core to live alongside it. Install travel via "
        "cmake --install on cpp/travel and switch this branch to "
        "find_package(travel) if you only have the ROS layer.")
endif()

# --- ament / ROS 2 dependencies --------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# --- Node ------------------------------------------------------------------
add_executable(travel_node src/travel_node.cpp)
ament_target_dependencies(travel_node
    rclcpp
    sensor_msgs
    std_msgs
    pcl_conversions
)
target_link_libraries(travel_node
    travel::travel_core
    ${PCL_LIBRARIES}
)
target_compile_definitions(travel_node PRIVATE TRAVEL_USE_ROS2_LOGGING)

# --- Install ---------------------------------------------------------------
install(TARGETS travel_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
