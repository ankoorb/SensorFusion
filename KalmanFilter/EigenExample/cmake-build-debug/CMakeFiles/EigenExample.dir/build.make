# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/EigenExample.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/EigenExample.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/EigenExample.dir/flags.make

CMakeFiles/EigenExample.dir/main.cpp.o: CMakeFiles/EigenExample.dir/flags.make
CMakeFiles/EigenExample.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/EigenExample.dir/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EigenExample.dir/main.cpp.o -c /Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample/main.cpp

CMakeFiles/EigenExample.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EigenExample.dir/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample/main.cpp > CMakeFiles/EigenExample.dir/main.cpp.i

CMakeFiles/EigenExample.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EigenExample.dir/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample/main.cpp -o CMakeFiles/EigenExample.dir/main.cpp.s

CMakeFiles/EigenExample.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/EigenExample.dir/main.cpp.o.requires

CMakeFiles/EigenExample.dir/main.cpp.o.provides: CMakeFiles/EigenExample.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/EigenExample.dir/build.make CMakeFiles/EigenExample.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/EigenExample.dir/main.cpp.o.provides

CMakeFiles/EigenExample.dir/main.cpp.o.provides.build: CMakeFiles/EigenExample.dir/main.cpp.o


# Object files for target EigenExample
EigenExample_OBJECTS = \
"CMakeFiles/EigenExample.dir/main.cpp.o"

# External object files for target EigenExample
EigenExample_EXTERNAL_OBJECTS =

EigenExample: CMakeFiles/EigenExample.dir/main.cpp.o
EigenExample: CMakeFiles/EigenExample.dir/build.make
EigenExample: CMakeFiles/EigenExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable EigenExample"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/EigenExample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/EigenExample.dir/build: EigenExample

.PHONY : CMakeFiles/EigenExample.dir/build

CMakeFiles/EigenExample.dir/requires: CMakeFiles/EigenExample.dir/main.cpp.o.requires

.PHONY : CMakeFiles/EigenExample.dir/requires

CMakeFiles/EigenExample.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/EigenExample.dir/cmake_clean.cmake
.PHONY : CMakeFiles/EigenExample.dir/clean

CMakeFiles/EigenExample.dir/depend:
	cd /Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample /Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample /Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample/cmake-build-debug /Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample/cmake-build-debug /Users/ankoor/Desktop/Courses/KalmanFilter/EigenExample/cmake-build-debug/CMakeFiles/EigenExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/EigenExample.dir/depend

