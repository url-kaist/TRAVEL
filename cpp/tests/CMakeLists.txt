cmake_minimum_required(VERSION 3.16)
project(travel_tests LANGUAGES CXX)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Two integration paths, mirroring cpp/examples/CMakeLists.txt:
#  (1) Built as part of the cpp/travel tree with TRAVEL_BUILD_TESTS=ON:
#      `travel::travel_core` is already defined.
#  (2) Built standalone after `cmake --install` of cpp/travel: discover via
#      find_package(travel).
if (NOT TARGET travel::travel_core)
    find_package(travel REQUIRED)
endif()

add_executable(regression_kitti regression_kitti.cpp)
target_link_libraries(regression_kitti PRIVATE travel::travel_core)
