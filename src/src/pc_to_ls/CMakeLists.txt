cmake_minimum_required(VERSION 3.8)
project(pc_to_ls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pc_to_ls src/pc_to_ls.cpp)
target_link_libraries(pc_to_ls m)
ament_target_dependencies(pc_to_ls rclcpp tf2 tf2_ros tf2_sensor_msgs sensor_msgs geometry_msgs tf2_geometry_msgs)

install(TARGETS pc_to_ls
        DESTINATION lib/${PROJECT_NAME})

ament_package()
