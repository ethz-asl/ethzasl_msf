cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
rosbuild_genmsg()

add_definitions (-std=c++0x -march=native)
set(ROS_COMPILE_FLAGS "-W -Wall")# -Wextra -pedantic")

#add_definitions (-DDEBUGPUBLISH) #publish the state after each propagation step for full rate plotting

#FIND EIGEN
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

rosbuild_add_library(pose_distorter src/msf_distort/PoseDistorter.cc)

add_subdirectory(src/pose_msf)
add_subdirectory(src/pose_pressure_msf)
add_subdirectory(src/position_msf)
add_subdirectory(src/position_pose_msf)
add_subdirectory(src/spherical_msf)

rosbuild_add_executable(test_distort src/test/test_distort.cc)
target_link_libraries(test_distort pose_distorter)

#rosbuild_add_executable(test_tmp src/test/tmp_devel.cc)

