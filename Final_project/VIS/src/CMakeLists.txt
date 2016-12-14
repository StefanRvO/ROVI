#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
find_package(OpenCV REQUIRED)


# And now we add any targets that we want
add_library(vision LineFinding.cpp helper_funcs.cpp Vision.cpp)
add_executable	(marker1_test main_mark1.cpp)

add_executable	(marker2_test main_mark2.cpp)
target_link_libraries(marker2_test vision ${OpenCV_LIBS})
target_link_libraries(marker1_test vision ${OpenCV_LIBS})
