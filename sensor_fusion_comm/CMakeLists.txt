cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion_comm)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_message_files(
  FILES
  DoubleArrayStamped.msg
  DoubleMatrixStamped.msg
  ExtEkf.msg
  ExtState.msg
  PointWithCovarianceStamped.msg
)

#uncomment if you have defined services
add_service_files(
  FILES
  InitScale.srv
  InitHeight.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime geometry_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
