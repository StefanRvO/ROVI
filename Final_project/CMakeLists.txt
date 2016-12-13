cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(vision)

find_package(OpenCV REQUIRED)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

SET( CMAKE_CXX_FLAGS  "	${CMAKE_CXX_FLAGS} -std=c++11 -Ofast -Wall -Wextra -Wpointer-arith -ffunction-sections -fdata-sections -Wcast-qual -g -Wno-unused-local-typedefs -Wno-unused-parameter" )

add_subdirectory("VIS")
add_subdirectory("ROB")

