cmake_minimum_required(VERSION 2.8.3)
project(servo_check)

include_directories(${PROJECT_SOURCE_DIR}/dynamixel_sdk)

find_library(libdxl dxl_x64_c PATHS ${PROJECT_SOURCE_DIR})
add_executable(servo_check servo_check.c)
target_link_libraries(servo_check ${libdxl})
