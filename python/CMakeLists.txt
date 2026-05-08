cmake_minimum_required(VERSION 3.16...3.31)
project(travel_seg_pybind LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG master
)
FetchContent_MakeAvailable(pybind11)

set(PYBIND11_NEWPYTHON ON)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

# Dual-mode integration:
#   In-tree dev (`pip install -e python/` from the repo): use the local
#   `cpp/travel/` tree directly via add_subdirectory.
#   Out-of-tree (`pip install travel-seg` from PyPI sdist): the `cpp/`
#   sibling directory is not present, so fetch the released TRAVEL repo
#   and consume its `cpp/travel` subdir via FetchContent SOURCE_SUBDIR.
if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/travel/CMakeLists.txt)
    message(STATUS "Building travel_seg pybind module against local cpp/travel tree")
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/travel
                     ${CMAKE_CURRENT_BINARY_DIR}/travel_core)
else()
    cmake_minimum_required(VERSION 3.18)
    message(STATUS "Out-of-tree build: fetching TRAVEL v${CMAKE_PROJECT_VERSION} from GitHub")
    FetchContent_Declare(
        ext_travel_core PREFIX travel_core
        GIT_REPOSITORY https://github.com/url-kaist/TRAVEL
        GIT_TAG main
        SOURCE_SUBDIR cpp/travel
    )
    FetchContent_MakeAvailable(ext_travel_core)
endif()

# Compiled extension. The leading underscore keeps it private — `__init__.py`
# re-exports the public surface so users can write `import travel_seg as ts`
# and never see the binding module directly.
set(OUTPUT_MODULE_NAME _travel_seg)

pybind11_add_module(${OUTPUT_MODULE_NAME}
    travel_seg/pybind/travel_pybind.cpp
)
target_link_libraries(${OUTPUT_MODULE_NAME} PUBLIC travel::travel_core)

# Silence third-party warnings (PCL / Eigen) inside the binding shim.
target_compile_options(${OUTPUT_MODULE_NAME} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-w>
    $<$<CXX_COMPILER_ID:Clang>:-w>
    $<$<CXX_COMPILER_ID:AppleClang>:-w>
    $<$<CXX_COMPILER_ID:MSVC>:/w>
)

# pybind11 + clang quirk; harmless on GCC.
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"
    OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    target_compile_options(${OUTPUT_MODULE_NAME} PUBLIC -fsized-deallocation)
endif()

# Install the .so inside the package directory so it is importable as
# `travel_seg._travel_seg`.
install(TARGETS ${OUTPUT_MODULE_NAME} DESTINATION travel_seg)
