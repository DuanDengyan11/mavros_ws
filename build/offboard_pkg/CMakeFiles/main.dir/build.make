# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tutu/FControl/mavros_ws/src/offboard_pkg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tutu/FControl/mavros_ws/build/offboard_pkg

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/src/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/main.cpp.o: /home/tutu/FControl/mavros_ws/src/offboard_pkg/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tutu/FControl/mavros_ws/build/offboard_pkg/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/src/main.cpp.o"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/main.cpp.o -c /home/tutu/FControl/mavros_ws/src/offboard_pkg/src/main.cpp

CMakeFiles/main.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/main.cpp.i"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tutu/FControl/mavros_ws/src/offboard_pkg/src/main.cpp > CMakeFiles/main.dir/src/main.cpp.i

CMakeFiles/main.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/main.cpp.s"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tutu/FControl/mavros_ws/src/offboard_pkg/src/main.cpp -o CMakeFiles/main.dir/src/main.cpp.s

CMakeFiles/main.dir/src/TaskManager.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/TaskManager.cpp.o: /home/tutu/FControl/mavros_ws/src/offboard_pkg/src/TaskManager.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tutu/FControl/mavros_ws/build/offboard_pkg/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/main.dir/src/TaskManager.cpp.o"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/TaskManager.cpp.o -c /home/tutu/FControl/mavros_ws/src/offboard_pkg/src/TaskManager.cpp

CMakeFiles/main.dir/src/TaskManager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/TaskManager.cpp.i"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tutu/FControl/mavros_ws/src/offboard_pkg/src/TaskManager.cpp > CMakeFiles/main.dir/src/TaskManager.cpp.i

CMakeFiles/main.dir/src/TaskManager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/TaskManager.cpp.s"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tutu/FControl/mavros_ws/src/offboard_pkg/src/TaskManager.cpp -o CMakeFiles/main.dir/src/TaskManager.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/main.cpp.o" \
"CMakeFiles/main.dir/src/TaskManager.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: CMakeFiles/main.dir/src/main.cpp.o
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: CMakeFiles/main.dir/src/TaskManager.cpp.o
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: CMakeFiles/main.dir/build.make
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/libtf.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/libtf2_ros.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/libactionlib.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/libmessage_filters.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/libroscpp.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/libtf2.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/librosconsole.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/librostime.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /opt/ros/noetic/lib/libcpp_common.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: /usr/lib/x86_64-linux-gnu/libGeographic.so
/home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tutu/FControl/mavros_ws/build/offboard_pkg/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: /home/tutu/FControl/mavros_ws/devel/.private/offboard_pkg/lib/offboard_pkg/main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/tutu/FControl/mavros_ws/build/offboard_pkg && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tutu/FControl/mavros_ws/src/offboard_pkg /home/tutu/FControl/mavros_ws/src/offboard_pkg /home/tutu/FControl/mavros_ws/build/offboard_pkg /home/tutu/FControl/mavros_ws/build/offboard_pkg /home/tutu/FControl/mavros_ws/build/offboard_pkg/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

