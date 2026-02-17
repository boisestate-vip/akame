cmake_minimum_required(VERSION 3.8)
project(f710)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(f710 src/main.cpp src/f710.cpp)
ament_target_dependencies(f710 rclcpp geometry_msgs)

install(TARGETS f710
        DESTINATION lib/${PROJECT_NAME})

ament_package()
