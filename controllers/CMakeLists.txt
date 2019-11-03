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

generate_dynamic_reconfigure_options(cfg/setSIT1FLC.cfg cfg/setDIT1FLC.cfg cfg/setSIT2FLC.cfg cfg/setDIT2FLC.cfg cfg/setFNN.cfg cfg/setFNN_IT2.cfg cfg/setDNN.cfg cfg/setPID.cfg cfg/setPIDsimple.cfg cfg/setTrajectory.cfg cfg/setNoise.cfg cfg/setSafeY6.cfg cfg/setSafeIntelDrone.cfg)

catkin_package()

include_directories(include "/opt/ros/kinetic/include" ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

LINK_DIRECTORIES("/usr/local/lib")

add_executable(q_learning src/q_learning.cpp)
target_link_libraries(q_learning ${catkin_LIBRARIES})
add_dependencies(q_learning controllers_generate_messages_cpp controllers_gencfg)

add_executable(si_it2_flc src/SI-IT2-FLC.cpp)
target_link_libraries(si_it2_flc ${catkin_LIBRARIES})
add_dependencies(si_it2_flc controllers_generate_messages_cpp controllers_gencfg)

add_executable(di_it2_flc src/DI-IT2-FLC.cpp)
target_link_libraries(di_it2_flc ${catkin_LIBRARIES})
add_dependencies(di_it2_flc controllers_generate_messages_cpp controllers_gencfg)

add_executable(di_it2_flc_fm src/DI-IT2-FLC-FM.cpp)
target_link_libraries(di_it2_flc_fm ${catkin_LIBRARIES})
add_dependencies(di_it2_flc_fm controllers_generate_messages_cpp controllers_gencfg)

add_executable(di_t1_flc_fm src/DI-T1-FLC-FM.cpp)
target_link_libraries(di_t1_flc_fm ${catkin_LIBRARIES})
add_dependencies(di_t1_flc_fm controllers_generate_messages_cpp controllers_gencfg)

add_executable(si_it2_flc_fm src/SI-IT2-FLC-FM.cpp)
target_link_libraries(si_it2_flc_fm ${catkin_LIBRARIES})
add_dependencies(si_it2_flc_fm controllers_generate_messages_cpp controllers_gencfg)

add_executable(si_t1_flc src/SI-T1-FLC.cpp)
target_link_libraries(si_t1_flc ${catkin_LIBRARIES})
add_dependencies(si_t1_flc controllers_generate_messages_cpp controllers_gencfg)

add_executable(si_t1_flc_fm src/SI-T1-FLC-FM.cpp)
target_link_libraries(si_t1_flc_fm ${catkin_LIBRARIES})
add_dependencies(si_t1_flc_fm controllers_generate_messages_cpp controllers_gencfg)

add_executable(fnn src/FNN.cpp)
target_link_libraries(fnn ${catkin_LIBRARIES})
add_dependencies(fnn controllers_generate_messages_cpp controllers_gencfg)

add_executable(fnn_it2 src/FNN-IT2.cpp)
target_link_libraries(fnn_it2 ${catkin_LIBRARIES})
add_dependencies(fnn_it2 controllers_generate_messages_cpp controllers_gencfg)

add_executable(pid src/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})
add_dependencies(pid controllers_generate_messages_cpp controllers_gencfg)

add_executable(trajectory src/trajectory.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES})
add_dependencies(trajectory controllers_generate_messages_cpp controllers_gencfg)

add_executable(noise src/noise.cpp)
target_link_libraries(noise ${catkin_LIBRARIES})
add_dependencies(noise controllers_generate_messages_cpp controllers_gencfg)

add_executable(logger src/logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES})
add_dependencies(logger controllers_generate_messages_cpp controllers_gencfg)

add_executable(teleop_key src/teleopKey.cpp)
target_link_libraries(teleop_key ${catkin_LIBRARIES})
add_dependencies(teleop_key controllers_generate_messages_cpp controllers_gencfg)

add_executable(safe_intel_drone src/safeIntelDrone.cpp)
target_link_libraries(safe_intel_drone ${catkin_LIBRARIES})
add_dependencies(safe_intel_drone controllers_generate_messages_cpp controllers_gencfg)
