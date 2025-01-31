cmake_minimum_required(VERSION 3.16)

project(open_vins CXX)

option(ILLIXR_INTEGRATION "Enable ILLIXR integration features" OFF)

if (NOT ILLIXR_INTEGRATION)
    message(FATAL_ERROR "This build script is for ILLIXR builds")
endif()

# By default we do not build with ROS, but you can disable this and just build as a library
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" OFF)
set(ILLIXR_ROOT "" CACHE PATH "Directory of ILLIXR includes")
option(ENABLE_TESTS "Enable building tests" OFF)
option(USE_ZED "Whether to build for the zed camera" OFF)
if (USE_ZED)
    add_compile_definitions(HAVE_ZED)
endif()
if (NOT ILLIXR_ROOT)
    message(FATAL_ERROR "ILLIXR_ROOT must be specified if ILLIXR_INTEGRATION is enabled.")
endif()

if (ENABLE_ROS)
    # We need c++14 for ROS2, thus just require it for everybody
    # NOTE: To future self, hope this isn't an issue...
    set(CMAKE_CXX_STANDARD 14)
else()
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



# Include libraries
# The OpenCV version needs to match the one used by cv_bridge otherwise you will get a segmentation fault!
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
if (CMAKE_MAJOR_VERSION VERSION_EQUAL 3 AND CMAKE_MINOR_VERSION GREATER 29)
    find_package(Boost REQUIRED CONFIG COMPONENTS system filesystem thread date_time)
else()
    find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
endif()
find_package(Ceres REQUIRED)
find_package(spdlog REQUIRED)

message(STATUS "OPENCV: ${OpenCV_VERSION} | BOOST: ${Boost_VERSION} | CERES: ${Ceres_VERSION} | SPDLOG: ${spdlog_VERSION}")

# If we will compile with aruco support
option(ENABLE_ARUCO_TAGS "Enable or disable aruco tag (disable if no contrib modules)" ON)
if (NOT ENABLE_ARUCO_TAGS)
    add_definitions(-DENABLE_ARUCO_TAGS=0)
    message(WARNING "DISABLING ARUCOTAG TRACKING!")
else ()
    add_definitions(-DENABLE_ARUCO_TAGS=1)
endif ()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DILLIXR_INTEGRATION -O3 -DNDEBUG -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -DILLIXR_INTEGRATION -ggdb -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

add_subdirectory(ov_core)
# add_subdirectory(ov_eval)
add_subdirectory(ov_msckf)
