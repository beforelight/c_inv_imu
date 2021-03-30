cmake_minimum_required(VERSION 3.17)

project(inv_imu)
add_compile_options(-Wall)
file(GLOB_RECURSE SOURCES *.c)
add_library(inv_imu STATIC ${SOURCES})
