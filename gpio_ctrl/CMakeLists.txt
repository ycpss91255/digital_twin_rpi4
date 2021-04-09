cmake_minimum_required(VERSION 3.0.2)
project(gpio_ctrl)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(
 INCLUDE_DIRS
  include
#  LIBRARIES gpio_ctrl
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
