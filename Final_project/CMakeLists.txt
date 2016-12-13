cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project(vision)
set (CMAKE_CXX_FLAGS "-Ofast -Wall -g -std=c++11")
find_package(OpenCV REQUIRED)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

SET( CMAKE_CXX_FLAGS  "	${CMAKE_CXX_FLAGS}	\
            		-Wno-error=unused-function  \
                   	 -Wno-error=unused-variable  \
			-std=c++11 		\
			-Ofast			\
		 	-Wall 			\
			-Wextra 		\
			-Wshadow 		\
			-Wpointer-arith 	\
			-ffunction-sections 	\
			-fdata-sections		\
			-Wcast-qual		\
			-Wno-unused-variable 	\
			-pedantic		\
			-Wunused-variable	\
			-g			\
" )

add_subdirectory("VIS")
#add_subdirectory("ROB")

