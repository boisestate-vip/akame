cmake_minimum_required(VERSION 3.8)
project(sim_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rewrite src/rewrite.cpp)
ament_target_dependencies(rewrite rclcpp sensor_msgs)

# install project
install(
   TARGETS rewrite
   DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY src launch rviz config world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
