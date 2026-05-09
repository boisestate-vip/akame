cmake_minimum_required(VERSION 3.8)
project(vive_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -O3)
endif()

# !!! remove this if you are manually setting the openvr !!!
# !!! directory below                                    !!!
if(NOT DEFINED ENV{OPENVR_DIRECTORY})
   message(SEND_ERROR "The OPENVR_DIRECTORY variable is not set in this shell. Please export it as the path to the openvr library that can be installed from https://github.com/ValveSoftware/openvr")
   return()
endif()

set(OpenVR_INCLUDE_DIR "$ENV{OPENVR_DIRECTORY}/headers")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenVR REQUIRED)


# ros stuff

add_executable(vive_tracker src/vive_tracker.cpp)
target_link_libraries(vive_tracker OpenVR::OpenVR)
ament_target_dependencies(vive_tracker rclcpp sensor_msgs 
                          geometry_msgs visualization_msgs 
                          nav_msgs std_msgs tf2)

install(
   TARGETS vive_tracker
   DESTINATION lib/${PROJECT_NAME})
install(
   DIRECTORY launch config
   DESTINATION share/${PROJECT_NAME})



ament_package()
