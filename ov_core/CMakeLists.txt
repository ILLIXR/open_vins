cmake_minimum_required(VERSION 3.16)

option(ILLIXR_INTEGRATION "Flag enabling integrated (non-standalone) mode" OFF)

# Project name
project(ov_core)

# Find catkin (the ROS build system)
#find_package(catkin QUIET COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge)


# display message to user
message(STATUS "EIGEN VERSION: " ${Eigen3_VERSION})
message(STATUS "OPENCV VERSION: " ${OpenCV_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})

# Describe catkin project
if(catkin_FOUND)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge
            INCLUDE_DIRS src
            LIBRARIES ov_core_lib
    )
else()
    message(WARNING "CATKIN NOT FOUND BUILDING WITHOUT ROS!")
endif()


# Enable compile optimizations

# Enable debug flags (use if you want to debug in gdb)
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Oy /O2 /fp:fast /ZI /Wall /we4700")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized")
endif()

# Include our header files
include_directories(
        src
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
     Boost::serialization
     spdlog::spdlog
     opencv_core
     opencv_imgproc
     opencv_calib3d
     Eigen3::Eigen
     ${catkin_LIBRARIES}
)

##################################################
# Make the core library
##################################################
add_library(ov_core_lib SHARED
            src/dummy.cpp
            src/init/InertialInitializer.cpp
            src/sim/BsplineSE3.cpp
            src/track/TrackBase.cpp
            src/track/TrackAruco.cpp
            src/track/TrackDescriptor.cpp
            src/track/TrackKLT.cpp
            src/track/TrackSIM.cpp
            src/types/Landmark.cpp
            src/feat/Feature.cpp
            src/feat/FeatureInitializer.cpp
)
if(ANDROID_BUILD)
    target_link_libraries(ov_core_lib
                          opencv_core
                          opencv_imgproc
                          opencv_tracking
                          opencv_imgcodecs
                          opencv_video
                          opencv_calib3d
                          opencv_highgui
                          opencv_features2d
                          opencv_aruco
                          opencv_dnn
                          opencv_flann
                          opencv_objdetect
                          Boost::chrono
                          Boost::thread
                          Boost::filesystem
                          Boost::atomic
                          android
                          Eigen3::Eigen
    )

    target_include_directories(ov_core_lib PRIVATE
                               ${OpenCV_LIBS}
                               ${jnigraphics-lib}
                               ${ANDROID_NDK}/sources/android/native_app_glue
    )
else()
    target_link_libraries(ov_core_lib ${thirdparty_libraries})
    target_include_directories(ov_core_lib PUBLIC src)
endif()
set_property(TARGET ov_core_lib PROPERTY CXX_STANDARD 17)

if(ILLIXR_INTEGRATION)
    if(ANDROID_BUILD)
        install(TARGETS ov_core_lib LIBRARY DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    else()
        install(TARGETS ov_core_lib DESTINATION lib)
    endif()
endif()
##################################################
# Make binary files!
##################################################
#if (catkin_FOUND)
#    add_executable(test_tracking src/test_tracking.cpp)
#    target_link_libraries(test_tracking ov_core_lib ${thirdparty_libraries})
#endif()
#
#
#add_executable(test_webcam src/test_webcam.cpp)
#target_link_libraries(test_webcam ov_core_lib ${thirdparty_libraries})
