cmake_minimum_required(VERSION 3.8)
project(tf_tune)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(tf_tune src/main.cpp)
ament_target_dependencies(
   tf_tune
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)

install(TARGETS tf_tune
        DESTINATION lib/${PROJECT_NAME})


ament_package()
