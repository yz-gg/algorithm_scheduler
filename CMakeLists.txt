cmake_minimum_required(VERSION 3.0.2)
project(scheduler)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
)

add_message_files(
  FILES
  ImagePoint.msg
  PipelineCenterlineObservation.msg
  PipelineLine.msg
)

add_service_files(
  FILES
  ControlModule.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scheduler_node
  src/scheduler_node.cpp
  src/mavros_command_interface.cpp
  src/pipeline_inspection_controller.cpp
  src/docking_controller.cpp
  src/simple_mission_controller.cpp
  src/mission_controller_factory.cpp
)
add_dependencies(scheduler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(scheduler_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/scheduler_launch.py
  scripts/pipeline_detection_stub.py
  scripts/external_pipeline_algorithm_bridge.py
  scripts/docking_vision_detector.py
  scripts/docking_light_strip_detector.py
  scripts/docking_apriltag_detector.py
  scripts/docking_data_logger.py
  scripts/pipeline_data_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS scheduler_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
