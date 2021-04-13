cmake_minimum_required(VERSION 3.0.2)
project(gpio_ctrl)

# add_library(YAML_LIB /usr/include/yaml-cpp/yaml.h )


# find_library(LIBPIGPIOD_IF2_LIBRARIES
#  NAMES
#   pigpiod_if2
# )

# test
# set(MCT_LIST
#   motor_control_test.cpp
#   ../src/motor_control.cpp
# )
# find_library(LIBPIGPIOD_IF2_LIBRARIES NAMES pigpiod_if2)

# add_executable(motor_control_test ${MCT_LIST})
# # target_link_libraries(motor_control_test ${catkin_LIBRARIES})
# target_link_libraries(motor_control_test ${catkin_LIBRARIES} ${LIBPIGPIOD_IF2_LIBRARIES})

# # test
# set(GC_LIST
#   gpio_ctrl_test.cpp
# )

# add_executable(gpio_ctrl_test ${GC_LIST})
# # target_link_libraries(gpio_ctrl_test ${catkin_LIBRARIES} -lpigpiod_if2)
# target_link_libraries(gpio_ctrl_test ${catkin_LIBRARIES} ${LIBPIGPIOD_IF2_LIBRARIES})
