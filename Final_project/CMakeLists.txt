cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(vision)
set (CMAKE_CXX_FLAGS "-Ofast -Wall -g -std=c++11")
find_package(OpenCV REQUIRED)

add_executable(main src/main.cpp)

target_link_libraries(main ${OpenCV_LIBS})

