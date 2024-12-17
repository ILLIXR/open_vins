cmake_minimum_required(VERSION 3.16)

option(ILLIXR_INTEGRATION "Flag enabling integrated (non-standalone) mode" OFF)

# Project name
project(ov_msckf)

# Include our cmake files
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

# Find catkin (the ROS build system)
#find_package(catkin QUIET COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge ov_core)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
if (CMAKE_VERSION VERSION_LESS "3.30.0")
    find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
else()
    find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time CONFIG)
endif()

# display message to user
message(STATUS "EIGEN VERSION: " ${Eigen3_VERSION})
message(STATUS "OPENCV VERSION: " ${OpenCV_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})
if(${OpenCV_VERSION_MAJOR} EQUAL 4)
    add_definitions(-DUSING_OPENCV4)
endif()

# Describe catkin project
if (catkin_FOUND)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge ov_core
            INCLUDE_DIRS src
            LIBRARIES ov_msckf_lib
    )
else()
    message(WARNING "CATKIN NOT FOUND BUILDING WITHOUT ROS!")
endif()

if (ILLIXR_INTEGRATION)
    # Enable compile optimizations
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DILLIXR_INTEGRATION -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
    # Enable debug flags (use if you want to debug in gdb)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized")
    include_directories(${ILLIXR_ROOT}
            src
            ${OpenCV_INCLUDE_DIR}
            ${EIGEN3_INCLUDE_DIR}
            ${Boost_INCLUDE_DIRS}
            ${catkin_INCLUDE_DIRS}
            )
else()
    # Enable compile optimizations
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
    # Enable debug flags (use if you want to debug in gdb)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized")
    # Include our header files
    include_directories(
            src
            ${OpenCV_INCLUDE_DIR}
            ${EIGEN3_INCLUDE_DIR}
            ${Boost_INCLUDE_DIRS}
            ${catkin_INCLUDE_DIRS}
    )

endif()


# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
)

# If we are not building with ROS then we need to manually link to its headers
# This isn't that elegant of a way, but this at least allows for building without ROS
# See this stackoverflow answer: https://stackoverflow.com/a/11217008/7718197
if (NOT catkin_FOUND)
    message(WARNING "MANUALLY LINKING TO OV_CORE LIBRARY....")
    include_directories(${ov_core_SOURCE_DIR}/src/)
    list(APPEND thirdparty_libraries ov_core_lib)
endif()

##################################################
# Make the core library
##################################################
list(APPEND library_source_files
        src/sim/Simulator.cpp
        src/state/State.cpp
        src/state/StateHelper.cpp
        src/state/Propagator.cpp
        src/core/VioManager.cpp
        src/update/UpdaterHelper.cpp
        src/update/UpdaterMSCKF.cpp
        src/update/UpdaterSLAM.cpp
)
if (catkin_FOUND)
    list(APPEND library_source_files
        src/core/RosVisualizer.cpp
    )
endif()
add_library(ov_msckf_lib SHARED ${library_source_files})
target_link_libraries(ov_msckf_lib ${thirdparty_libraries})
target_include_directories(ov_msckf_lib PUBLIC src)
set_property(TARGET ov_msckf_lib PROPERTY CXX_STANDARD 17)


##################################################
# Make binary files!
##################################################

if (ILLIXR_INTEGRATION)
    find_package(spdlog REQUIRED)
    set(PLUGIN_NAME plugin.openvins${ILLIXR_BUILD_SUFFIX})
    add_library(${PLUGIN_NAME} SHARED src/slam2.cpp)
    target_include_directories(${PLUGIN_NAME} PUBLIC ${ILLIXR_ROOT})
    target_link_libraries(${PLUGIN_NAME} ov_msckf_lib ${thirdparty_libraries} spdlog::spdlog)
    set_property(TARGET ${PLUGIN_NAME} PROPERTY CXX_STANDARD 17)
    install(TARGETS ov_msckf_lib DESTINATION lib)
    install(TARGETS ${PLUGIN_NAME} DESTINATION lib)
else()
    add_executable(run_illixr_msckf src/run_illixr_msckf.cpp)
    target_link_libraries(run_illixr_msckf ov_msckf_lib ${thirdparty_libraries})
    set_property(TARGET run_illixr_msckf PROPERTY CXX_STANDARD 17)
endif()
