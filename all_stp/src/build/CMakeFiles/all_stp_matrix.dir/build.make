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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nuc/workplace/all_stp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nuc/workplace/all_stp/build

# Include any dependencies generated for this target.
include CMakeFiles/all_stp_matrix.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/all_stp_matrix.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/all_stp_matrix.dir/flags.make

CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o: CMakeFiles/all_stp_matrix.dir/flags.make
CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o: ../src/all_stp_matrix.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nuc/workplace/all_stp/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o -c /home/nuc/workplace/all_stp/src/all_stp_matrix.cpp

CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/nuc/workplace/all_stp/src/all_stp_matrix.cpp > CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.i

CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/nuc/workplace/all_stp/src/all_stp_matrix.cpp -o CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.s

CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o.requires:
.PHONY : CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o.requires

CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o.provides: CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o.requires
	$(MAKE) -f CMakeFiles/all_stp_matrix.dir/build.make CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o.provides.build
.PHONY : CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o.provides

CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o.provides.build: CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o

# Object files for target all_stp_matrix
all_stp_matrix_OBJECTS = \
"CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o"

# External object files for target all_stp_matrix
all_stp_matrix_EXTERNAL_OBJECTS =

../bin/all_stp_matrix: CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o
../bin/all_stp_matrix: CMakeFiles/all_stp_matrix.dir/build.make
../bin/all_stp_matrix: CMakeFiles/all_stp_matrix.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/all_stp_matrix"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/all_stp_matrix.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/all_stp_matrix.dir/build: ../bin/all_stp_matrix
.PHONY : CMakeFiles/all_stp_matrix.dir/build

CMakeFiles/all_stp_matrix.dir/requires: CMakeFiles/all_stp_matrix.dir/src/all_stp_matrix.cpp.o.requires
.PHONY : CMakeFiles/all_stp_matrix.dir/requires

CMakeFiles/all_stp_matrix.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/all_stp_matrix.dir/cmake_clean.cmake
.PHONY : CMakeFiles/all_stp_matrix.dir/clean

CMakeFiles/all_stp_matrix.dir/depend:
	cd /home/nuc/workplace/all_stp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nuc/workplace/all_stp /home/nuc/workplace/all_stp /home/nuc/workplace/all_stp/build /home/nuc/workplace/all_stp/build /home/nuc/workplace/all_stp/build/CMakeFiles/all_stp_matrix.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/all_stp_matrix.dir/depend

