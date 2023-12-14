cmake_minimum_required(VERSION 3.0.2)
project(robot_description)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  joint_state_publisher
  robot_state_publisher
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_description
#  CATKIN_DEPENDS gazebo_ros joint_state_publisher robot_state_publisher roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)