cmake_minimum_required(VERSION 2.8.3)
project(msf_updates)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

#build with boost 1.6x
add_definitions(-DBOOST_NO_CXX11_VARIADIC_TEMPLATES)

find_package(catkin REQUIRED COMPONENTS roscpp msf_core geometry_msgs sensor_fusion_comm)

include_directories(include ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
    cfg/PosePressureSensor.cfg
    cfg/PositionPoseSensor.cfg
    cfg/SinglePoseSensor.cfg
    cfg/SinglePositionSensor.cfg
    cfg/SphericalPositionSensor.cfg
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp msf_core geometry_msgs sensor_fusion_comm
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
    LIBRARIES pose_distorter
)

add_library(pose_distorter src/msf_distort/PoseDistorter.cc)
target_link_libraries(pose_distorter ${catkin_LIBRARIES})

#just build the pose filter on the helicopters
if(EXISTS "${PROJECT_SOURCE_DIR}/COMPILE_ONLY_POSEFILTER")
add_subdirectory(src/pose_msf)
else()
add_subdirectory(src/pose_msf)
add_subdirectory(src/pose_pressure_msf)
add_subdirectory(src/position_msf)
add_subdirectory(src/position_pose_msf)
add_subdirectory(src/spherical_msf)
endif()

add_executable(test_distort src/test/test_distort.cc)
target_link_libraries(test_distort pose_distorter)
