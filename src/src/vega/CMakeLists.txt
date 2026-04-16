cmake_minimum_required(VERSION 3.8)
project(vega)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)

# the node
add_executable(vega src/vega.cpp)
ament_target_dependencies(vega rclcpp nav_msgs std_msgs geometry_msgs tf2
                                     builtin_interfaces visualization_msgs)

install(TARGETS vega
        DESTINATION lib/${PROJECT_NAME})


ament_package()
