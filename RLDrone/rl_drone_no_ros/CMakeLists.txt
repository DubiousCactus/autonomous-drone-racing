cmake_minimum_required(VERSION 2.8.3)
project(rl_drone)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(rl_control src/rl_control.cpp)
target_link_libraries(rl_control ${GAZEBO_LIBRARIES} pthread)
