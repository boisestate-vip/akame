cmake_minimum_required(VERSION 3.8)
project(simple_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# the node
add_executable(simple_map src/simple_map.cpp)
ament_target_dependencies(simple_map rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs
                                     builtin_interfaces tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs)

install(TARGETS simple_map
        DESTINATION lib/${PROJECT_NAME})

ament_package()
