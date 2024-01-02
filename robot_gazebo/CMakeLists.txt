cmake_minimum_required(VERSION 3.0.2)
project(robot_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  rospy
  std_msgs
  gazebo_ros
  geometry_msgs
  ackermann_msgs
  joint_state_publisher
  robot_state_publisher
  tf2_ros
  iss_manager
)

catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES robot_gazebo
 CATKIN_DEPENDS controller_manager roscpp rospy iss_manager
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/gazebo_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
