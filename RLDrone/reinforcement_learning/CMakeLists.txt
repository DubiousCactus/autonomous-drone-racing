cmake_minimum_required (VERSION 2.8)

# projectname is the same as the main-executable
project(reinforcement_learning)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_definitions('-g')
add_definitions('-Wall')
add_definitions('-std=c++11')

add_library(reinforcement_learning SHARED rl_control.cc)
target_link_libraries(reinforcement_learning ${GAZEBO_LIBRARIES})
#add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)

#add_custom_target(${PROJECT_NAME}-symlink ALL ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/${PROJECT_NAME} DEPENDS ${PROJECT_NAME})
#set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_SOURCE_DIR}/${PROJECT_NAME})
