cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # for scoped_lock
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# set(OpenCV_DIR "~/.local/lib/cmake/opencv4")
# find_package(OpenCV 4.2.0 REQUIRED)

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# find_package(orbslam3_msgs REQUIRED)

# These are required for ORB_SLAM3

# find_package(Eigen3 3.1.0 REQUIRED)

find_package(Sophus REQUIRED)
# set(Pangolin_DIR "~/.local/lib/cmake/Pangolin")
find_package(Pangolin REQUIRED)
# set(ORB_SLAM3_DIR "~/.local/lib/cmake/ORB_SLAM3")
find_package(ORB_SLAM3 REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

link_directories(
  include
)

# add_executable(mono
#   src/monocular/mono.cpp
#   src/monocular/monocular-slam-node.cpp
# )
# ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

# add_executable(rgbd
#   src/rgbd/rgbd.cpp
#   src/rgbd/rgbd-slam-node.cpp
# )
# ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)

# add_executable(stereo
#   src/stereo/stereo.cpp
#   src/stereo/stereo-slam-node.cpp
# )
# ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters
# ORB_SLAM3 Pangolin)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
target_include_directories(stereo-inertial PRIVATE src)

message(STATUS "ORB_SLAM3_LIBRARIES: ${ORB_SLAM3_LIBRARIES}")



# target_link_libraries(stereo-inertial
#   "~/multi-agent-robotics/ORB-SLAM3-STEREO-FIXED/lib/libORB_SLAM3.so"
#   # ${ORB_SLAM3_LIBS}
#   # ${Pangolin_LIBRARIES}
#   # ${OpenCV_LIBS}
#   # ${EIGEN3_LIBS}
#   # ${Sophus_LIBS}
# )

target_link_libraries(stereo-inertial ${ORB_SLAM3_LIBRARIES})




ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)
# ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge Pangolin)

# install(TARGETS mono rgbd stereo stereo-inertial

install(
  FILES ${ORB_SLAM3_LIBRARIES}
  DESTINATION lib/
)

install(TARGETS stereo-inertial
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

