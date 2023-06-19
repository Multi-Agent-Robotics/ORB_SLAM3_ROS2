cmake_minimum_required (VERSION 3.16.3 FATAL_ERROR) # default on Ubuntu 20.04
project (orbslam3_wrapper
    DESCRIPTION "ORB-SLAM3 ROS2 wrapper"
    LANGUAGES CXX)

add_executable (stereo-inertial "")

include (cmake/printers.cmake)
include (cmake/CPM.cmake)

# as the tag is in an unusual format, we need to explicitly specify the version
CPMAddPackage ("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")

# You should set the PYTHONPATH to your own python site-packages path
if (NOT ENV{PYTHONPATH})
    set (ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")
endif ()

# Generate symbols for LSP indexing
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

set (CMAKE_CXX_STANDARD 17) # for scoped_lock
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options (stereo-inertial PRIVATE -Wall -Wextra -Wpedantic)
endif ()

find_program (CCACHE_PROGRAM ccache)

if (CCACHE_PROGRAM)
    set (CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set (CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")

    # CMake 3.9+
    set (CMAKE_CUDA_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message (STATUS "Using ccache for compilation.")
endif ()

# ROS2 packages
find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (geometry_msgs REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (cv_bridge REQUIRED)
find_package (message_filters REQUIRED)
find_package (orbslam3_msgs REQUIRED)

find_package (OpenCV 4.2.0 REQUIRED)
find_package (Eigen3 3.1.0 REQUIRED)

find_package (orbslam3 CONFIG REQUIRED)

# as the tag is in an unusual format, we need to explicitly specify the version
CPMAddPackage ("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")

# print_all_variables_with_postfix (INCLUDE_DIRS)
target_sources (stereo-inertial PUBLIC
    src/stereo-inertial/stereo-inertial.cpp
    src/stereo-inertial/stereo-inertial-node.cpp
    src/stereo-inertial/stereo-inertial-node.hpp
)

target_include_directories (stereo-inertial
    PUBLIC
    src
    include
    ${orbslam3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features (stereo-inertial PUBLIC cxx_std_17)

get_target_property (SO_LOCATION orbslam3 IMPORTED_LOCATION_NOCONFIG)
message (STATUS "SO_LOCATION: ${SO_LOCATION}")

target_link_libraries (stereo-inertial
    orbslam3
    ${Pangolin_LIBRARIES}
)

target_compile_options (stereo-inertial PRIVATE
    -Wno-deprecated-declarations
)

ament_target_dependencies (stereo-inertial rclcpp sensor_msgs orbslam3_msgs geometry_msgs cv_bridge message_filters)

install (TARGETS stereo-inertial
    DESTINATION lib/${PROJECT_NAME}
)

# This must be the last line in the file
ament_package ()
