cmake_minimum_required(VERSION 3.8)
project(drum_driver)
set(CMAKE_VERBOSE_MAKEFILE TRUE)

# from unitree
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# from ros2
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Synexens sdk linking
set(UNITREE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(UNITREE_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}")
include_directories(${UNITREE_INCLUDE_DIR})
link_directories(${UNITREE_LIB_DIR})
#include_directories(include)
#link_directories(lib)

#if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64") 
#    set(EXTRA_LIBS libUnitreeMotorSDK_Arm64.so)
#else()
#    set(EXTRA_LIBS libUnitreeMotorSDK_Linux64.so)
#endif()

set(UNITREE_LIB libUnitreeMotorSDK.so)

add_executable(drum_driver src/main.cpp)
target_link_directories(drum_driver PRIVATE ${UNITREE_LIB_DIR})
target_link_libraries(drum_driver ${UNITREE_LIB})
ament_target_dependencies(drum_driver rclcpp std_msgs)

# preserve rpath for shared libraries
set_target_properties(drum_driver PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

target_include_directories(drum_driver PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)

install(TARGETS drum_driver
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ${UNITREE_LIB_DIR}/
        DESTINATION lib)

ament_package()
