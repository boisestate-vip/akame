cmake_minimum_required(VERSION 3.5)
project(central_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(central_controller_node src/central_controller.cpp)
ament_target_dependencies(
  central_controller_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS
  central_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
