cmake_minimum_required(VERSION 3.8)
project(regolith_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(regolith_driver src/main.cpp)
target_include_directories(regolith_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(regolith_driver rclcpp std_msgs)

install(TARGETS regolith_driver
        DESTINATION lib/${PROJECT_NAME})

ament_package()
