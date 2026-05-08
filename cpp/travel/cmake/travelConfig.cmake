get_filename_component(TRAVEL_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
list(PREPEND CMAKE_MODULE_PATH "${TRAVEL_CMAKE_DIR}")
include(CMakeFindDependencyMacro)

find_dependency(PCL REQUIRED COMPONENTS common io filters)
find_dependency(Boost REQUIRED COMPONENTS filesystem system)

include("${TRAVEL_CMAKE_DIR}/travelTargets.cmake")
