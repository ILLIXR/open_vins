cmake_minimum_required(VERSION 3.16)

# Project name
project(ov_core CXX)

if (NOT ILLIXR_INTEGRATION)
    # Include libraries
    # The OpenCV version needs to match the one used by cv_bridge otherwise you will get a segmentation fault!
    find_package(Eigen3 REQUIRED)
    find_package(OpenCV 4 QUIET)
    find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
    message(STATUS "OPENCV: " ${OpenCV_VERSION} " | BOOST: " ${Boost_VERSION})

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
if (CMAKE_VERSION VERSION_LESS "3.30.0")
    find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
else()
    find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time CONFIG)
endif()

    # If we will compile with aruco support
    option(ENABLE_ARUCO_TAGS "Enable or disable aruco tag (disable if no contrib modules)" ON)
    if (NOT ENABLE_ARUCO_TAGS)
        add_definitions(-DENABLE_ARUCO_TAGS=0)
        message(WARNING "DISABLING ARUCOTAG TRACKING!")
    else ()
        add_definitions(-DENABLE_ARUCO_TAGS=1)
    endif ()

    # Enable compile optimizations
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
    # Enable debug flags (use if you want to debug in gdb)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -ggdb -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

endif()
# check if we have our python libs files (will search for python3 then python2 installs)
# sudo apt-get install python-matplotlib python-numpy python-dev
# https://cmake.org/cmake/help/v3.10/module/FindPythonLibs.html
option(DISABLE_MATPLOTLIB "Disable or enable matplotlib plot scripts in ov_eval" OFF)
if (NOT DISABLE_MATPLOTLIB)
    find_package(Python3 QUIET COMPONENTS Interpreter Development NumPy)

    if (Python3_FOUND AND Python3_LIBRARIES AND Python3_NumPy_FOUND)
        add_definitions(-DHAVE_PYTHONLIBS=1)
        message(STATUS "PYTHON VERSION: " ${Python3_VERSION})
        message(STATUS "PYTHON INCLUDE: " ${Python3_INCLUDE_DIRS})
        message(STATUS "PYTHON LIBRARIES: " ${Python3_LIBRARIES})
        include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
        list(APPEND thirdparty_libraries ${Python3_LIBRARIES})
    endif ()
endif()


# Find our ROS version!
# NOTE: Default to using the ROS1 package if both are in our enviroment
# NOTE: https://github.com/romainreignier/share_ros1_ros2_lib_demo
if (ENABLE_ROS)
    find_package(catkin QUIET COMPONENTS roscpp)
    find_package(ament_cmake QUIET)
    if (catkin_FOUND)
        message(STATUS "ROS *1* version found, building ROS1.cmake")
        include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
    elseif (ament_cmake_FOUND)
        message(STATUS "ROS *2* version found, building ROS2.cmake")
        include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
    else()
        message(FATAL_ERROR "Unexpected configuration")
    endif ()
else ()
    message(STATUS "No ROS versions found or building with ROS disabled, building ROS1.cmake without ROS")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
endif()
