cmake_minimum_required(VERSION 3.0.2)
project(social_nav_metrics)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  pedsim_msgs
  pedsim_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES social_nav_metrics
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

