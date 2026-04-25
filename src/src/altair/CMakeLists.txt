cmake_minimum_required(VERSION 3.8)
project(altair)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# the node
add_executable(altair src/altair.cpp)
ament_target_dependencies(altair rclcpp nav_msgs std_msgs geometry_msgs visualization_msgs
                                 tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs)

install(TARGETS altair
        DESTINATION lib/${PROJECT_NAME})

ament_package()
