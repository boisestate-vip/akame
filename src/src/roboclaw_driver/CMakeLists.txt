cmake_minimum_required(VERSION 3.8)
project(roboclaw_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(roboclaw_driver src/main.cpp)
ament_target_dependencies(roboclaw_driver rclcpp std_msgs)

install(TARGETS roboclaw_driver
        DESTINATION lib/${PROJECT_NAME})

ament_package()
