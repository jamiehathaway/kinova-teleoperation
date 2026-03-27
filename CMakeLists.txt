cmake_minimum_required(VERSION 3.10)
project(kinova_teleoperation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(pinocchio REQUIRED)

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

add_executable(kinova_teleop
  src/api_setup.cpp
  src/arm_core_functions.cpp
  src/leader.cpp)
target_include_directories(kinova_teleop PRIVATE ${KORTEX_INCLUDE_DIRS})
target_compile_definitions(kinova_teleop PRIVATE _OS_UNIX DURATION=${DURATION})
target_link_libraries(kinova_teleop KortexApiCpp pthread)

add_executable(kinova_torque_control_test
  src/api_setup.cpp
  src/arm_core_functions.cpp
  src/actuator_csv_logger.cpp
  src/motor_model_utils.cpp
  src/torque_control_test.cpp)
target_include_directories(kinova_torque_control_test PRIVATE ${KORTEX_INCLUDE_DIRS})
target_compile_definitions(kinova_torque_control_test PRIVATE _OS_UNIX DURATION=${DURATION})
target_link_libraries(kinova_torque_control_test KortexApiCpp pthread)
target_link_libraries(kinova_torque_control_test pinocchio::pinocchio)
ament_target_dependencies(kinova_torque_control_test
  ament_index_cpp
  kdl_parser
  urdf)

add_executable(kinova_moveit_current_logger
  src/api_setup.cpp
  src/moveit_random_pose_current_logger.cpp)
target_include_directories(kinova_moveit_current_logger PRIVATE ${KORTEX_INCLUDE_DIRS})
target_compile_definitions(kinova_moveit_current_logger PRIVATE _OS_UNIX)
target_link_libraries(kinova_moveit_current_logger KortexApiCpp
  pthread
  moveit_ros_planning_interface::moveit_move_group_interface)
ament_target_dependencies(kinova_moveit_current_logger
  rclcpp
  sensor_msgs
  moveit_ros_planning_interface)

add_executable(kinova_motor_calibration
  src/api_setup.cpp
  src/arm_core_functions.cpp  
  src/actuator_csv_logger.cpp
  src/motor_model_utils.cpp
  src/motor_calibration.cpp
)
target_include_directories(kinova_motor_calibration PRIVATE ${KORTEX_INCLUDE_DIRS})
target_compile_definitions(kinova_motor_calibration PRIVATE _OS_UNIX)
target_link_libraries(kinova_motor_calibration KortexApiCpp pthread)
target_link_libraries(kinova_motor_calibration pinocchio::pinocchio)
ament_target_dependencies(kinova_motor_calibration
  ament_index_cpp
  kdl_parser
  urdf)

install(TARGETS
  kinova_teleop
  kinova_torque_control_test
  kinova_moveit_current_logger
  kinova_motor_calibration
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
