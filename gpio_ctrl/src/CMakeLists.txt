cmake_minimum_required(VERSION 3.0.2)
project(gpio_ctrl)

# test
set(GC_LIST
  gpio_ctrl.cpp
)
find_library(LIBPIGPIOD_IF2_LIBRARIES NAMES pigpiod_if2)

add_executable(gpio_ctrl ${GC_LIST})
# target_link_libraries(gpio_ctrl ${catkin_LIBRARIES} -lpigpiod_if2)
target_link_libraries(gpio_ctrl ${catkin_LIBRARIES} ${LIBPIGPIOD_IF2_LIBRARIES})
