cmake_minimum_required(VERSION 3.8)
project(vive_to_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(vive_to_odom src/main.cpp)
ament_target_dependencies(vive_to_odom rclcpp sensor_msgs nav_msgs geometry_msgs tf2
                          tf2_ros tf2_sensor_msgs tf2_geometry_msgs)

install(TARGETS vive_to_odom
        DESTINATION lib/${PROJECT_NAME})

ament_package()
