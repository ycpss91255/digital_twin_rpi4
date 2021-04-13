cmake_minimum_required(VERSION 3.0.2)
project(gpio_ctrl)

find_library(LIBPIGPIOD_IF2_LIBRARIES
 NAMES
  pigpiod_if2
)

set(MC_LIST
  motor_control.cpp
)
find_library(LIBPIGPIOD_IF2_LIBRARIES NAMES pigpiod_if2)

add_executable(motor_control ${MC_LIST})
target_link_libraries(motor_control ${catkin_LIBRARIES} ${LIBPIGPIOD_IF2_LIBRARIES})

# # test
# set(GC_LIST
#   gpio_ctrl_test.cpp
# )

# add_executable(gpio_ctrl_test ${GC_LIST})
# # target_link_libraries(gpio_ctrl_test ${catkin_LIBRARIES} -lpigpiod_if2)
# target_link_libraries(gpio_ctrl_test ${catkin_LIBRARIES} ${LIBPIGPIOD_IF2_LIBRARIES})
