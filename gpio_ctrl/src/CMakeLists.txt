cmake_minimum_required(VERSION 3.0.2)
project(gpio_ctrl)

find_library(LIBPIGPIOD_IF2_LIBRARIES
  NAMES
    pigpiod_if2
)

# find python libraries
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

# populate matplotlib repository
include(FetchContent)
FetchContent_Declare(
    matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

set(MC_LIST
  motor_control.cpp
  node_handle.cpp
)
add_executable(motor_control ${MC_LIST})
target_link_libraries(motor_control
  ${catkin_LIBRARIES}
  ${LIBPIGPIOD_IF2_LIBRARIES}
  ${PYTHON_LIBRARIES}
  Python3::NumPy
)
