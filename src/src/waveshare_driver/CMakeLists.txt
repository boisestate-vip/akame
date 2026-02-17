cmake_minimum_required(VERSION 3.8)
project(waveshare_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(waveshare_driver src/main.cpp)
ament_target_dependencies(waveshare_driver rclcpp geometry_msgs)

install(TARGETS waveshare_driver
        DESTINATION lib/${PROJECT_NAME})

ament_package()
