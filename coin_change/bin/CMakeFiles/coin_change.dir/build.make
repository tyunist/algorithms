# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tynguyen/algorithms/coin_change

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tynguyen/algorithms/coin_change/bin

# Include any dependencies generated for this target.
include CMakeFiles/coin_change.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/coin_change.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/coin_change.dir/flags.make

CMakeFiles/coin_change.dir/src/coin_change.cpp.o: CMakeFiles/coin_change.dir/flags.make
CMakeFiles/coin_change.dir/src/coin_change.cpp.o: ../src/coin_change.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tynguyen/algorithms/coin_change/bin/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/coin_change.dir/src/coin_change.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/coin_change.dir/src/coin_change.cpp.o -c /home/tynguyen/algorithms/coin_change/src/coin_change.cpp

CMakeFiles/coin_change.dir/src/coin_change.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/coin_change.dir/src/coin_change.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/tynguyen/algorithms/coin_change/src/coin_change.cpp > CMakeFiles/coin_change.dir/src/coin_change.cpp.i

CMakeFiles/coin_change.dir/src/coin_change.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/coin_change.dir/src/coin_change.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/tynguyen/algorithms/coin_change/src/coin_change.cpp -o CMakeFiles/coin_change.dir/src/coin_change.cpp.s

CMakeFiles/coin_change.dir/src/coin_change.cpp.o.requires:
.PHONY : CMakeFiles/coin_change.dir/src/coin_change.cpp.o.requires

CMakeFiles/coin_change.dir/src/coin_change.cpp.o.provides: CMakeFiles/coin_change.dir/src/coin_change.cpp.o.requires
	$(MAKE) -f CMakeFiles/coin_change.dir/build.make CMakeFiles/coin_change.dir/src/coin_change.cpp.o.provides.build
.PHONY : CMakeFiles/coin_change.dir/src/coin_change.cpp.o.provides

CMakeFiles/coin_change.dir/src/coin_change.cpp.o.provides.build: CMakeFiles/coin_change.dir/src/coin_change.cpp.o

# Object files for target coin_change
coin_change_OBJECTS = \
"CMakeFiles/coin_change.dir/src/coin_change.cpp.o"

# External object files for target coin_change
coin_change_EXTERNAL_OBJECTS =

coin_change: CMakeFiles/coin_change.dir/src/coin_change.cpp.o
coin_change: CMakeFiles/coin_change.dir/build.make
coin_change: CMakeFiles/coin_change.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable coin_change"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/coin_change.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/coin_change.dir/build: coin_change
.PHONY : CMakeFiles/coin_change.dir/build

CMakeFiles/coin_change.dir/requires: CMakeFiles/coin_change.dir/src/coin_change.cpp.o.requires
.PHONY : CMakeFiles/coin_change.dir/requires

CMakeFiles/coin_change.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/coin_change.dir/cmake_clean.cmake
.PHONY : CMakeFiles/coin_change.dir/clean

CMakeFiles/coin_change.dir/depend:
	cd /home/tynguyen/algorithms/coin_change/bin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tynguyen/algorithms/coin_change /home/tynguyen/algorithms/coin_change /home/tynguyen/algorithms/coin_change/bin /home/tynguyen/algorithms/coin_change/bin /home/tynguyen/algorithms/coin_change/bin/CMakeFiles/coin_change.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/coin_change.dir/depend

