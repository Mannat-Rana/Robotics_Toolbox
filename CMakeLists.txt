cmake_minimum_required(VERSION 3.1)
project(Robotics_Toolbox)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(src/*)

add_library(RoboticsToolbox src/RoboticsToolbox.cpp)
add_executable(main src/main.cpp)
target_link_libraries(main RoboticsToolbox)