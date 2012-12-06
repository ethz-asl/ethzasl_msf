cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#gencfg()
rosbuild_genmsg()

add_definitions (-Wall -std=c++0x)

# get eigen
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#rosbuild_add_executable(pose_sensor src/main.cpp)
#set_property(TARGET pose_sensor PROPERTY COMPILE_DEFINITIONS POSE_MEAS)
#rosbuild_add_compile_flags(pose_sensor "-O3")

#rosbuild_add_executable(position_sensor src/main.cpp)
#set_property(TARGET position_sensor PROPERTY COMPILE_DEFINITIONS POSITION_MEAS)
#rosbuild_add_compile_flags(position_sensor "-O3")

rosbuild_add_executable(msf_viconpos_sensor src/main.cpp)
set_property(TARGET msf_viconpos_sensor PROPERTY COMPILE_DEFINITIONS VICONPOS_MEAS)
##rosbuild_add_compile_flags(msf_viconpos_sensor "-O3")

#rosbuild_add_executable(prototype src/prototyping.cc)
