cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES  Bbox.msg GatePrediction.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


catkin_install_python(PROGRAMS nodes/detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
