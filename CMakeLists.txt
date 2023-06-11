cmake_minimum_required(VERSION 3.20)

project("UsbHost" C CXX ASM)

include(cmake/st-project.cmake)

include_directories(libs)

add_executable(${PROJECT_NAME})
add_st_target_properties(${PROJECT_NAME})