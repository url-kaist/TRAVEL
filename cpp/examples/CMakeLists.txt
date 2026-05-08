cmake_minimum_required(VERSION 3.16)
project(travel_examples LANGUAGES CXX)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Two integration paths:
#   (1) Built as part of the cpp/travel tree (TRAVEL_BUILD_EXAMPLES=ON):
#       the `travel::travel_core` target already exists, just use it.
#   (2) Built standalone after `cmake --install` of cpp/travel: discover via
#       find_package.
if (NOT TARGET travel::travel_core)
    find_package(travel REQUIRED)
endif()

# Materialize MPI::MPI_C before PCL on platforms where libpcl-dev is built
# against VTK 9.1 (Ubuntu 22.04+); see cpp/tests/CMakeLists.txt for
# context. No-op where PCL's VTK doesn't require MPI.
find_package(MPI)
find_package(PCL REQUIRED COMPONENTS common io filters)

add_executable(run_travel_kitti src/run_travel_kitti.cpp)
target_link_libraries(run_travel_kitti
    PRIVATE
        travel::travel_core
        ${PCL_LIBRARIES}
)
target_include_directories(run_travel_kitti SYSTEM PRIVATE
    ${PCL_INCLUDE_DIRS}
)
