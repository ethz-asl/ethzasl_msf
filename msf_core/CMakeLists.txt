cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Generate dynamic parameters

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
#rosbuild_genmsg()

add_definitions (-Wall -std=c++0x)

# get eigen
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_subdirectory(src/lib)

#common commands for building c++ executables and libraries
#rosbuild_add_library(msf_core src/MSF_Core.cpp src/measurement.cpp src/state.cpp)
#rosbuild_add_compile_flags(msf_core "-O3")

#rosbuild_add_executable(staticstatelist src/test_staticstatelist.cc)
#rosbuild_add_gtest_build_flags(staticstatelist)

#rosbuild_add_executable(test_tmptestbed src/test_tmptestbed.cc)
#rosbuild_add_executable(test_msfcore src/test_msf_core.cc)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#rosbuild_add_gtest(test_staticstatelist src/test_staticstatelist.cc)
#rosbuild_link_boost(test_staticstatelist thread)

rosbuild_add_library(similaritytransform src/similaritytransform.cpp)
rosbuild_add_executable(test_similaritytransform src/test_similaritytransform.cpp)
target_link_libraries(test_similaritytransform similaritytransform)