cmake_minimum_required(VERSION 3.16.3) # default on Ubuntu 20.04
project(orbslam3_wrapper LANGUAGES CXX)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")

include(cmake/printers.cmake)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Generate symbols for LSP indexing
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17) # for scoped_lock
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_program(CCACHE_PROGRAM ccache)

if(CCACHE_PROGRAM)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")

    # CMake 3.9+
    set(CMAKE_CUDA_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "Using ccache for compilation.")
endif()

# set(3RDPARTY_INSTALL_DIR "/mnt/c/Users/jensj/source/repos/ORB-SLAM3-STEREO-FIXED/3rdparty")
# list(PREPEND CMAKE_MODULE_PATH ${3RDPARTY_INSTALL_DIR}/install)

# message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

# set(OpenCV_DIR "~/.local/lib/cmake/opencv4")
# find_package(OpenCV 4.2.0 REQUIRED)

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(orbslam3_msgs REQUIRED)

# # set(orbslam3_DIR ${3RDPARTY_INSTALL_DIR}/install/lib/cmake/orbslam3)
# if(NOT orbslam3_DIR)
#     message(FATAL_ERROR "orbslam3_DIR not found. Please set the path with -Dorbslam3_DIR=/path/to/orbslam3")
# endif()

set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)

find_package(orbslam3 REQUIRED)

add_executable(stereo-inertial "")

target_sources(stereo-inertial PRIVATE
    src/stereo-inertial/stereo-inertial.cpp
    src/stereo-inertial/stereo-inertial-node.cpp
)
target_include_directories(stereo-inertial PRIVATE src include)

target_link_options(stereo-inertial PRIVATE
    $<$<CONFIG:Release>:-03 -march=native>
    $<$<CONFIG:Debug>:-O0 -g>
)

# target_link_libraries(stereo-inertial PUBLIC orbslam3::orbslam3)

ament_target_dependencies(stereo-inertial rclcpp sensor_msgs orbslam3_msgs cv_bridge orbslam3)

print_target_included_directories(stereo-inertial)

# install(TARGETS mono rgbd stereo stereo-inertial
# install(
# FILES ${ORB_SLAM3_LIBRARIES}
# DESTINATION lib/
# )
install(TARGETS stereo-inertial
    DESTINATION lib/${PROJECT_NAME}
)

# This must be the last line in the file
ament_package()
