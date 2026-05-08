cmake_minimum_required(VERSION 3.8)
project(synexens_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Synexens sdk linking
set(SYNEXENS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(SYNEXENS_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}")
include_directories(${SYNEXENS_INCLUDE_DIR})
link_directories(${SYNEXENS_LIB_DIR})

set(SYNEXENS_LIB csreconstruction2.0 SynexensSDK)

add_executable(synexens_node src/synexens_node.cpp)
target_link_libraries(synexens_node ${SYNEXENS_LIB})
ament_target_dependencies(synexens_node rclcpp sensor_msgs)

# preserve rpath for shared libraries
set_target_properties(synexens_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

target_include_directories(synexens_node PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)

#install(DIRECTORY 
#   DESTINATION share/${PROJECT_NAME})

install(TARGETS synexens_node
   DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ${SYNEXENS_LIB_DIR}/
   DESTINATION lib)
install(DIRECTORY launch config
   DESTINATION share/${PROJECT_NAME})


ament_package()
