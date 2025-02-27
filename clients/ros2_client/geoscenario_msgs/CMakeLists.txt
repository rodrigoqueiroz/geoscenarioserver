cmake_minimum_required(VERSION 3.16)
project(geoscenario_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

set (MSGS
  msg/Pedestrian.msg
  msg/Tick.msg
  msg/Vehicle.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS}
  DEPENDENCIES std_msgs geometry_msgs geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
