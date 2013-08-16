cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion_comm)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_message_files(
  FILES
  DoubleArrayStamped.msg
  ExtEkf.msg
  ExtState.msg
  PointWithCovarianceStamped.msg
)

#uncomment if you have defined services
#add_service_files(
#  FILES
  # TODO: List your msg files here
#)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)
# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS
    CATKIN_DEPENDS  geometry_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
