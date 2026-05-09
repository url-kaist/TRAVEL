get_filename_component(TRAVEL_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
list(PREPEND CMAKE_MODULE_PATH "${TRAVEL_CMAKE_DIR}")
include(CMakeFindDependencyMacro)

# Eigen is the sole system dependency from the v1.1 release onward.
# (Pre-1.1, the core depended on PCL + Boost; that was dropped to make
# Linux / macOS / Windows wheels feasible without per-platform PCL builds.)
find_dependency(Eigen3 NO_MODULE)

include("${TRAVEL_CMAKE_DIR}/travelTargets.cmake")
