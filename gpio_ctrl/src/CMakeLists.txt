cmake_minimum_required(VERSION 3.0.2)
project(gpio_ctrl)

find_library(LIBPIGPIOD_IF2_LIBRARIES
 NAMES
  pigpiod_if2
)

# set(MCT_LIST
#   motor_control.cpp
# )
# add_executable(motor_control_test ${MCT_LIST})
# target_link_libraries(motor_control_test ${catkin_LIBRARIES} ${LIBPIGPIOD_IF2_LIBRARIES})

set(MC_LIST
  motor_control.cpp
  node_handle.cpp
)
add_executable(motor_control ${MC_LIST})
target_link_libraries(motor_control ${catkin_LIBRARIES} ${LIBPIGPIOD_IF2_LIBRARIES})
