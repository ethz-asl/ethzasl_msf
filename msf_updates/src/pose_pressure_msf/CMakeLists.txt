cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_add_executable(pose_pressure_sensor main.cpp)
rosbuild_add_compile_flags(pose_pressure_sensor "-O3")

