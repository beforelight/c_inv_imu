cmake_minimum_required(VERSION 3.17)

project(inv_imu)
add_definitions(-D_C_INV_IMU_DEV_)
add_compile_options(-Wall)
file(GLOB_RECURSE SOURCES *.c)
add_library(inv_imu STATIC ${SOURCES})
