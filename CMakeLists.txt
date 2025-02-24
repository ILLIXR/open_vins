cmake_minimum_required(VERSION 3.16)

project(open_vins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(ILLIXR_INTEGRATION "Enable ILLIXR integration features" OFF)

if (NOT ILLIXR_INTEGRATION)
    message(FATAL_ERROR "This build script is for ILLIXR builds")
endif()

# By default we do not build with ROS, but you can disable this and just build as a library
set(ILLIXR_ROOT "" CACHE PATH "Directory of ILLIXR includes")

option(USE_ZED "Whether to build for the zed camera" OFF)
if (USE_ZED)
    add_compile_definitions(HAVE_ZED)
endif()
if (NOT ILLIXR_ROOT)
    message(FATAL_ERROR "ILLIXR_ROOT must be specified if ILLIXR_INTEGRATION is enabled.")
endif()

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
if (CMAKE_VERSION VERSION_LESS "3.30.0")
    find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
else()
    find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time CONFIG)
endif()
find_package(spdlog REQUIRED)

add_subdirectory(ov_core)
# add_subdirectory(ov_eval)
add_subdirectory(ov_msckf)
