cmake_minimum_required(VERSION 3.16...3.31)
project(travel VERSION 1.1.0 LANGUAGES CXX)

option(USE_CCACHE "Build using Ccache if found on the path" ON)
option(TRAVEL_BUILD_EXAMPLES "Build standalone examples in cpp/examples" OFF)
option(TRAVEL_BUILD_TESTS "Build regression / unit tests in cpp/tests" OFF)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (USE_CCACHE)
    find_program(CCACHE_PROGRAM ccache)
    if (CCACHE_PROGRAM)
        set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    endif()
endif()

include(GNUInstallDirs)
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --- Dependencies ----------------------------------------------------------
# Eigen is the only system dep the core needs. PCL and Boost were dropped in
# the v1.1 release (cpp/travel/core/travel/types.hpp now provides the small
# slice we used to borrow from PCL). Eigen is header-only and ships
# everywhere we care about — apt's libeigen3-dev, brew's eigen, vcpkg's
# eigen3 — so cibuildwheel can produce a Linux/macOS/Windows wheel matrix
# without per-platform package contortions.
find_package(Eigen3 REQUIRED NO_MODULE)

# --- Library target --------------------------------------------------------
# The TRAVEL core is header-only (templated on PointType), so we expose it as
# an INTERFACE library. Downstream consumers (ros/, cpp/examples/, python/)
# link against `travel::travel_core` and inherit the include paths.
set(TARGET_NAME travel_core)
add_library(${TARGET_NAME} INTERFACE)
add_library(travel::travel_core ALIAS ${TARGET_NAME})

target_include_directories(${TARGET_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${TARGET_NAME} INTERFACE
    Eigen3::Eigen
)

# --- Examples (opt-in) -----------------------------------------------------
if (TRAVEL_BUILD_EXAMPLES)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../examples ${CMAKE_CURRENT_BINARY_DIR}/examples)
endif()

# --- Tests (opt-in) --------------------------------------------------------
if (TRAVEL_BUILD_TESTS)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../tests ${CMAKE_CURRENT_BINARY_DIR}/tests)
endif()

# --- Install ---------------------------------------------------------------
install(TARGETS ${TARGET_NAME}
    EXPORT ${PROJECT_NAME}-export
)

install(EXPORT ${PROJECT_NAME}-export
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}Targets.cmake
)

install(DIRECTORY core/travel DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES cmake/${PROJECT_NAME}Config.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
