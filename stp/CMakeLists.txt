cmake_minimum_required (VERSION 2.8)
project (stp)

MESSAGE(STATUS "CMAKE_CURRENT_BINARY_DIR: " ${CMAKE_CURRENT_BINARY_DIR})
MESSAGE(STATUS "CMAKE_FILES_DIRECTORY: " ${CMAKE_FILES_DIRECTORY})
MESSAGE(STATUS "CMAKE_BINARY_DIR: " ${CMAKE_BINARY_DIR})
# Set root repository variable
set(BIN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin")
# CMAKE_CURRENT_SOURCE_DIR is the directory in which CMake file is stored

# Set repository for all cmake generated files
set(CMAKE_FILES_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

# This is used to find opencv library package that is required to build our program
#find_package (OpenCV REQUIRED)

# Store binary file in bin_path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${BIN_PATH}")

# Add headers
#set(HEADER_PATH ${CMAKE_CURRENT_SOURCE_DIR}")
#file(GLOB HEADER_FILES "${HEADER_PATH}/*.h")

#include_directories(${HEADER_PATH})

# Create executable file. if in order to create rrackObject, we need a number of .cpp files, just use: add_executable (carTrack *.cpp)
add_executable (stp_matrix src/stp_matrix.cpp)
add_executable (prim src/prim.cpp)
add_executable (stp_linkedlist src/stp_linkedlist.cpp src/graph_class.cpp)

# In order to run executable file, we need to target Opencv library link to it
#target_link_libraries (stp_linkedlist ${OpenCV_LIBS})

# Test Script to manage program's variables 
enable_testing()
add_test(stp_linkedlist
	${BIN_PATH}/stp
	    )

