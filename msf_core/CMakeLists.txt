cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

# Set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Generate dynamic parameters

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
#rosbuild_genmsg()

add_definitions (-Wall -std=c++11 -Wno-unused-parameter)

# Enable timing the core
add_definitions (-DENABLE_MSF_TIMING)

# Get Eigen
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_subdirectory(src/lib)

rosbuild_add_library(similaritytransform src/similaritytransform.cc)
rosbuild_add_executable(test_similaritytransform src/test/test_similaritytransform.cc)
target_link_libraries(test_similaritytransform similaritytransform)
