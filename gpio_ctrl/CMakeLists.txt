cmake_minimum_required(VERSION 3.0.2)
project(gpio_ctrl)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  MotorFB.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
 INCLUDE_DIRS
  include
 CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_runtime
)

include_directories(
# include
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
# add_subdirectory(test)
