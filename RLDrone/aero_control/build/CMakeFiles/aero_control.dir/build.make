# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.14.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.14.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control/build

# Include any dependencies generated for this target.
include CMakeFiles/aero_control.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/aero_control.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/aero_control.dir/flags.make

CMakeFiles/aero_control.dir/aero_control.cc.o: CMakeFiles/aero_control.dir/flags.make
CMakeFiles/aero_control.dir/aero_control.cc.o: ../aero_control.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/aero_control.dir/aero_control.cc.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aero_control.dir/aero_control.cc.o -c /Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control/aero_control.cc

CMakeFiles/aero_control.dir/aero_control.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aero_control.dir/aero_control.cc.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control/aero_control.cc > CMakeFiles/aero_control.dir/aero_control.cc.i

CMakeFiles/aero_control.dir/aero_control.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aero_control.dir/aero_control.cc.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control/aero_control.cc -o CMakeFiles/aero_control.dir/aero_control.cc.s

# Object files for target aero_control
aero_control_OBJECTS = \
"CMakeFiles/aero_control.dir/aero_control.cc.o"

# External object files for target aero_control
aero_control_EXTERNAL_OBJECTS =

libaero_control.dylib: CMakeFiles/aero_control.dir/aero_control.cc.o
libaero_control.dylib: CMakeFiles/aero_control.dir/build.make
libaero_control.dylib: /usr/local/lib/libSimTKsimbody.3.6.dylib
libaero_control.dylib: /usr/local/lib/libdart.6.7.3.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_client.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_gui.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_sensors.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_rendering.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_physics.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_ode.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_transport.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_msgs.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_util.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_common.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_gimpact.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_opcode.dylib
libaero_control.dylib: /usr/local/Cellar/gazebo9/9.8.0/lib/libgazebo_opende_ou.dylib
libaero_control.dylib: /usr/local/lib/libboost_thread-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_system-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_filesystem-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_program_options-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_regex-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_iostreams-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_date_time-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_chrono-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_atomic-mt.dylib
libaero_control.dylib: /usr/local/lib/libprotobuf.dylib
libaero_control.dylib: /usr/local/Cellar/sdformat6/6.2.0_1/lib/libsdformat.dylib
libaero_control.dylib: /usr/local/Cellar/ogre1.9/1.9-20160714-108ab0bcc69603dba32c0ffd4bbbc39051f421c9_8/lib/libOgreMain.dylib
libaero_control.dylib: /usr/local/lib/libboost_thread-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_date_time-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_system-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_atomic-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_chrono-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_chrono-mt.dylib
libaero_control.dylib: /usr/local/Cellar/ogre1.9/1.9-20160714-108ab0bcc69603dba32c0ffd4bbbc39051f421c9_8/lib/libOgreTerrain.dylib
libaero_control.dylib: /usr/local/Cellar/ogre1.9/1.9-20160714-108ab0bcc69603dba32c0ffd4bbbc39051f421c9_8/lib/libOgrePaging.dylib
libaero_control.dylib: /usr/local/lib/libignition-transport4.4.0.0.dylib
libaero_control.dylib: /usr/local/lib/libignition-msgs1.1.0.0.dylib
libaero_control.dylib: /usr/local/lib/libignition-common1.1.1.1.dylib
libaero_control.dylib: /usr/local/lib/libignition-fuel_tools1.1.2.0.dylib
libaero_control.dylib: /usr/local/lib/libboost_filesystem-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_program_options-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_regex-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_iostreams-mt.dylib
libaero_control.dylib: /usr/local/lib/libprotobuf.dylib
libaero_control.dylib: /usr/local/Cellar/sdformat6/6.2.0_1/lib/libsdformat.dylib
libaero_control.dylib: /usr/local/Cellar/ogre1.9/1.9-20160714-108ab0bcc69603dba32c0ffd4bbbc39051f421c9_8/lib/libOgreMain.dylib
libaero_control.dylib: /usr/local/Cellar/ogre1.9/1.9-20160714-108ab0bcc69603dba32c0ffd4bbbc39051f421c9_8/lib/libOgreTerrain.dylib
libaero_control.dylib: /usr/local/Cellar/ogre1.9/1.9-20160714-108ab0bcc69603dba32c0ffd4bbbc39051f421c9_8/lib/libOgrePaging.dylib
libaero_control.dylib: /usr/local/lib/libSimTKmath.3.6.dylib
libaero_control.dylib: /usr/local/lib/libSimTKcommon.3.6.dylib
libaero_control.dylib: /usr/local/lib/libdart-external-odelcpsolver.6.7.3.dylib
libaero_control.dylib: /usr/local/lib/libccd.2.0.dylib
libaero_control.dylib: /usr/lib/libm.dylib
libaero_control.dylib: /usr/local/Cellar/fcl/0.5.0_1/lib/libfcl.dylib
libaero_control.dylib: /usr/local/Cellar/assimp/4.1.0/lib/libassimp.dylib
libaero_control.dylib: /usr/local/lib/libboost_system-mt.dylib
libaero_control.dylib: /usr/local/lib/libboost_filesystem-mt.dylib
libaero_control.dylib: /usr/local/lib/liboctomap.1.9.0.dylib
libaero_control.dylib: /usr/local/lib/liboctomath.1.9.0.dylib
libaero_control.dylib: /usr/local/lib/libboost_regex-mt.dylib
libaero_control.dylib: /usr/local/lib/libprotobuf.dylib
libaero_control.dylib: /usr/local/lib/libignition-math4.4.0.0.dylib
libaero_control.dylib: /usr/local/lib/libuuid.dylib
libaero_control.dylib: /usr/local/lib/libuuid.dylib
libaero_control.dylib: /usr/local/lib/libswscale.dylib
libaero_control.dylib: /usr/local/lib/libswscale.dylib
libaero_control.dylib: /usr/local/lib/libavdevice.dylib
libaero_control.dylib: /usr/local/lib/libavdevice.dylib
libaero_control.dylib: /usr/local/lib/libavformat.dylib
libaero_control.dylib: /usr/local/lib/libavformat.dylib
libaero_control.dylib: /usr/local/lib/libavcodec.dylib
libaero_control.dylib: /usr/local/lib/libavcodec.dylib
libaero_control.dylib: /usr/local/lib/libavutil.dylib
libaero_control.dylib: /usr/local/lib/libavutil.dylib
libaero_control.dylib: CMakeFiles/aero_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libaero_control.dylib"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aero_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/aero_control.dir/build: libaero_control.dylib

.PHONY : CMakeFiles/aero_control.dir/build

CMakeFiles/aero_control.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/aero_control.dir/cmake_clean.cmake
.PHONY : CMakeFiles/aero_control.dir/clean

CMakeFiles/aero_control.dir/depend:
	cd /Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control /Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control /Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control/build /Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control/build /Users/theomorales/Code/racing-drone-vision/RLDrone/aero_control/build/CMakeFiles/aero_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/aero_control.dir/depend
