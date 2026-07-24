cmake_minimum_required(VERSION 3.10)
project(kinova_teleoperation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
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

add_executable(demo_leader_follower
  src/util/api_setup.cpp
  src/util/arm_core_functions.cpp
  src/demo_leader_follower.cpp
)
target_include_directories(demo_leader_follower PRIVATE ${KORTEX_INCLUDE_DIRS})
target_compile_definitions(demo_leader_follower PRIVATE _OS_UNIX DURATION=${DURATION})
target_link_libraries(demo_leader_follower
  KortexApiCpp
  pthread
)

add_executable(leader
  src/util/api_setup.cpp
  src/util/arm_core_functions.cpp
  src/util/actuator_csv_logger.cpp
  src/util/motor_model_utils.cpp
  src/leader.cpp
)
target_include_directories(leader PRIVATE ${KORTEX_INCLUDE_DIRS})
target_compile_definitions(leader PRIVATE _OS_UNIX DURATION=${DURATION})
target_link_libraries(leader
  KortexApiCpp
  pthread
  pinocchio::pinocchio
)
ament_target_dependencies(leader
  rclcpp
  geometry_msgs
  ament_index_cpp
)

add_executable(random_pose_collect
  src/util/api_setup.cpp
  src/random_pose_collect.cpp
)
target_include_directories(random_pose_collect PRIVATE ${KORTEX_INCLUDE_DIRS})
target_compile_definitions(random_pose_collect PRIVATE _OS_UNIX)
target_link_libraries(random_pose_collect
  KortexApiCpp
  pthread
  moveit_ros_planning_interface::moveit_move_group_interface
)
ament_target_dependencies(random_pose_collect
  rclcpp
  sensor_msgs
  moveit_ros_planning_interface
)

add_executable(motor_calibration_collect
  src/util/api_setup.cpp
  src/util/arm_core_functions.cpp  
  src/util/actuator_csv_logger.cpp
  src/util/motor_model_utils.cpp
  src/motor_calibration_collect.cpp
)
target_include_directories(motor_calibration_collect PRIVATE ${KORTEX_INCLUDE_DIRS})
target_compile_definitions(motor_calibration_collect PRIVATE _OS_UNIX)
target_link_libraries(motor_calibration_collect
  KortexApiCpp
  pthread
  pinocchio::pinocchio
)
ament_target_dependencies(motor_calibration_collect
  ament_index_cpp
)

install(TARGETS
  demo_leader_follower
  leader
  random_pose_collect
  motor_calibration_collect
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
