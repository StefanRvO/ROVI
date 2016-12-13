#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)
find_package(OpenCV REQUIRED)


# And now we add any targets that we want
add_executable	(LineFinding LineFinding.cpp helper_funcs.cpp)
target_link_libraries(LineFinding ${OpenCV_LIBS})
