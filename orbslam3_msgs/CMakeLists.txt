cmake_minimum_required(VERSION 3.5)
project(orbslam3_msgs)

# Generate symbols for LSP indexing
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# get all files with .msg extension using a file glob
file(GLOB_RECURSE MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
list(LENGTH MSG_FILES MSG_FILES_LENGTH)
file(GLOB_RECURSE SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")
list(LENGTH SRV_FILES SRV_FILES_LENGTH)

# print each file to the console
message(STATUS "Found ${MSG_FILES_LENGTH} message files:")
foreach(MSG_FILE ${MSG_FILES})
  message(STATUS "+ ${MSG_FILE}")
endforeach()

message(STATUS "Found ${SRC_FILES_LENGTH} service files:")
foreach(SRV_FILE ${SRV_FILES})
  message(STATUS "+ ${SRV_FILE}")
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces std_msgs
)

# set(MSG_FILES
#   "msg/Keypoint.msg"
#   "msg/Descriptors.msg"
#   "msg/OrbFeatures.msg"
# )


# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${MSG_FILES}
#   ${SRV_FILES}
#   DEPENDENCIES builtin_interfaces std_msgs
#   ADD_LINTER_TESTS
# )
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
