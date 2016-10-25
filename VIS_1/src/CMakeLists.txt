cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(vision)

find_package(OpenCV REQUIRED)

add_executable(vision main.cpp DftUtilities.cpp)

target_link_libraries(Exercise3 ${OpenCV_LIBS})
