#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
find_package(OpenCV REQUIRED)


# And now we add any targets that we want
add_executable	(LineFinding LineFinding.cpp helper_funcs.cpp)
add_executable	(circles main.cpp)
target_link_libraries(LineFinding ${OpenCV_LIBS})
target_link_libraries(circles ${OpenCV_LIBS})
