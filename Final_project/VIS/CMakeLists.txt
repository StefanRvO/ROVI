CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (VIS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set (VIS_HEADERDIR "${VIS_DIR}/include")
set (VIS_HEADERS ${VIS_HEADERDIR}/helper_funcs.hpp)
set (VIS_HEADERS ${VIS_HEADERS} ${VIS_HEADERDIR}/LineFinding.hpp)
set (VIS_HEADERS ${VIS_HEADERS} ${VIS_HEADERDIR}/Vision.hpp)
include_directories(${VIS_HEADERDIR})
add_subdirectory("src")
