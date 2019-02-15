cmake_minimum_required(VERSION 2.8.3)
project(controllers)

set (CMAKE_CXX_STANDARD 11)
SET(CPACK_GENERATOR "TGZ")

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
#file(GLOB_RECURSE EXTRA_FILES */*)
#add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs sensor_msgs geometry_msgs tf dynamic_reconfigure)
find_package(gazebo REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(cfg/setSIT1FLC.cfg cfg/setDIT1FLC.cfg cfg/setSIT2FLC.cfg cfg/setDIT2FLC.cfg cfg/setPID.cfg cfg/setPIDsimple.cfg cfg/setTrajectory.cfg cfg/setNoise.cfg)

catkin_package()

include_directories(include "/opt/ros/kinetic/include" ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(di_it2_flc_fm src/DI-IT2-FLC-FM.cpp)
target_link_libraries(di_it2_flc_fm ${catkin_LIBRARIES})
add_dependencies(di_it2_flc_fm controllers_generate_messages_cpp controllers_gencfg)

add_executable(di_t1_flc_fm src/DI-T1-FLC-FM.cpp)
target_link_libraries(di_t1_flc_fm ${catkin_LIBRARIES})
add_dependencies(di_t1_flc_fm controllers_generate_messages_cpp controllers_gencfg)

add_executable(si_it2_flc_fm src/SI-IT2-FLC-FM.cpp)
target_link_libraries(si_it2_flc_fm ${catkin_LIBRARIES})
add_dependencies(si_it2_flc_fm controllers_generate_messages_cpp controllers_gencfg)

add_executable(si_t1_flc_fm src/SI-T1-FLC-FM.cpp)
target_link_libraries(si_t1_flc_fm ${catkin_LIBRARIES})
add_dependencies(si_t1_flc_fm controllers_generate_messages_cpp controllers_gencfg)

add_executable(pid src/PID.cpp)
#target_link_libraries(pid ${catkin_LIBRARIES} tensorflow)
target_link_libraries(pid ${catkin_LIBRARIES})
add_dependencies(pid controllers_generate_messages_cpp controllers_gencfg)

add_executable(trajectory src/trajectory.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES})
add_dependencies(trajectory controllers_generate_messages_cpp controllers_gencfg)

add_executable(noise src/noise.cpp)
target_link_libraries(noise ${catkin_LIBRARIES})
add_dependencies(noise controllers_generate_messages_cpp controllers_gencfg)

#add_executable(logger src/logger.cpp)
#target_link_libraries(logger ${catkin_LIBRARIES})
#add_dependencies(logger controllers_generate_messages_cpp controllers_gencfg)
