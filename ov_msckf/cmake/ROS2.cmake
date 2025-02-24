cmake_minimum_required(VERSION 3.3)

# Find ROS build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ov_core REQUIRED)
find_package(ov_init REQUIRED)

# Describe ROS project
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ON)
if (NOT ENABLE_ROS)
    message(FATAL_ERROR "Build with ROS1.cmake if you don't have ROS.")
endif ()
add_definitions(-DROS_AVAILABLE=2)

if (ILLIXR_INTEGRATION)
    # Include our header files
    include_directories(
            src
            ${ILLIXR_ROOT}
            ${EIGEN3_INCLUDE_DIR}
            ${Boost_INCLUDE_DIRS}
            ${CERES_INCLUDE_DIRS}
    )
else()
    # Include our header files
    include_directories(
            src
            ${EIGEN3_INCLUDE_DIR}
            ${Boost_INCLUDE_DIRS}
            ${CERES_INCLUDE_DIRS}
    )
endif()

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${CERES_LIBRARIES}
        ${OpenCV_LIBRARIES}
)
list(APPEND ament_libraries
        rclcpp
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        cv_bridge
        image_transport
        ov_core
        ov_init
)

##################################################
# Make the shared library
##################################################

list(APPEND LIBRARY_SOURCES
        src/dummy.cpp
        src/sim/Simulator.cpp
        src/state/State.cpp
        src/state/StateHelper.cpp
        src/state/Propagator.cpp
        src/core/VioManager.cpp
        src/core/VioManagerHelper.cpp
        src/update/UpdaterHelper.cpp
        src/update/UpdaterMSCKF.cpp
        src/update/UpdaterSLAM.cpp
        src/update/UpdaterZeroVelocity.cpp
)
list(APPEND LIBRARY_SOURCES src/ros/ROS2Visualizer.cpp src/ros/ROSVisualizerHelper.cpp)
file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")
add_library(ov_msckf_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
ament_target_dependencies(ov_msckf_lib ${ament_libraries})
target_link_libraries(ov_msckf_lib ${thirdparty_libraries})
target_include_directories(ov_msckf_lib PUBLIC src/)
install(TARGETS ov_msckf_lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include
)
install(DIRECTORY src/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
ament_export_include_directories(include)
ament_export_libraries(ov_msckf_lib)

##################################################
# Make binary files!
##################################################

add_executable(run_subscribe_msckf src/run_subscribe_msckf.cpp)
ament_target_dependencies(run_subscribe_msckf ${ament_libraries})
target_link_libraries(run_subscribe_msckf ov_msckf_lib ${thirdparty_libraries})
install(TARGETS run_subscribe_msckf DESTINATION lib/${PROJECT_NAME})

if (ILLIXR_INTEGRATION)
    find_package(spdlog REQUIRED)
    string(TOLOWER "${CMAKE_BUILD_TYPE}" lower_type)
    if(lower_type MATCHES "debug")
        set(ILLIXR_BUILD_SUFFIX ".dbg")
    elseif(lower_type MATCHES "release")
        set(ILLIXR_BUILD_SUFFIX ".opt")
    endif()

    set(PLUGIN_NAME plugin.openvins${ILLIXR_BUILD_SUFFIX})
    add_library(${PLUGIN_NAME} SHARED src/slam2.cpp)
    target_include_directories(${PLUGIN_NAME} PUBLIC ${ILLIXR_ROOT})
    target_link_libraries(${PLUGIN_NAME} ov_msckf_lib ov_core_lib ${thirdparty_libraries} spdlog::spdlog)
    set_property(TARGET ${PLUGIN_NAME} PROPERTY CXX_STANDARD 17)
    install(TARGETS ov_msckf_lib DESTINATION lib)
    install(TARGETS ${PLUGIN_NAME} DESTINATION lib)
else()
    add_executable(run_illixr_msckf src/run_illixr_msckf.cpp)
    target_link_libraries(run_illixr_msckf ov_msckf_lib ${thirdparty_libraries})
    set_property(TARGET run_illixr_msckf PROPERTY CXX_STANDARD 17)
endif()

if (ENABLE_TESTS)
    add_executable(test_sim_meas src/test_sim_meas.cpp)
    ament_target_dependencies(test_sim_meas ${ament_libraries})
    target_link_libraries(test_sim_meas ov_msckf_lib ${thirdparty_libraries})
    install(TARGETS test_sim_meas DESTINATION lib/${PROJECT_NAME})

    add_executable(test_sim_repeat src/test_sim_repeat.cpp)
    ament_target_dependencies(test_sim_repeat ${ament_libraries})
    target_link_libraries(test_sim_repeat ov_msckf_lib ${thirdparty_libraries})
    install(TARGETS test_sim_repeat DESTINATION lib/${PROJECT_NAME})
endif()

# Install launch and config directories
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY ../config/ DESTINATION share/${PROJECT_NAME}/config/)

# finally define this as the package
ament_package()
