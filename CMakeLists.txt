cmake_minimum_required(VERSION 3.10)
project(kinova_teleoperation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Set DURATION variable with default value of 20
if(NOT DEFINED DURATION)
  set(DURATION 20)
endif()

set(KORTEX_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/kortex_api/include
  ${CMAKE_CURRENT_SOURCE_DIR}/kortex_api/include/client
  ${CMAKE_CURRENT_SOURCE_DIR}/kortex_api/include/client_stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/kortex_api/include/common
  ${CMAKE_CURRENT_SOURCE_DIR}/kortex_api/include/google
  ${CMAKE_CURRENT_SOURCE_DIR}/kortex_api/include/messages
  ${CMAKE_CURRENT_SOURCE_DIR}/kortex_api/include/participant
  ${CMAKE_CURRENT_SOURCE_DIR}/kortex_api/include/server
  ${CMAKE_CURRENT_SOURCE_DIR}/kortex_api/include/server_stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/kortex_api/include/util
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/kortex_api/lib/release)

add_executable(kinova_example
  api_setup.cpp
  leader.cpp
  arm_core_functions.cpp)

target_include_directories(kinova_example PRIVATE ${KORTEX_INCLUDE_DIRS})
target_compile_definitions(kinova_example PRIVATE _OS_UNIX DURATION=${DURATION})
target_link_libraries(kinova_example KortexApiCpp pthread)

add_executable(kinova_moveit_current_logger
  api_setup.cpp
  moveit_random_pose_current_logger.cpp)

target_include_directories(kinova_moveit_current_logger PRIVATE ${KORTEX_INCLUDE_DIRS})
target_compile_definitions(kinova_moveit_current_logger PRIVATE _OS_UNIX)
target_link_libraries(kinova_moveit_current_logger
  KortexApiCpp
  pthread
  moveit_ros_planning_interface::moveit_move_group_interface)
ament_target_dependencies(kinova_moveit_current_logger
  rclcpp
  sensor_msgs
  moveit_ros_planning_interface)

install(TARGETS
  kinova_example
  kinova_moveit_current_logger
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
