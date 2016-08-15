cmake_minimum_required(VERSION 2.8.3)
project(msf_eval)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#build with boost 1.6x
add_definitions(-DBOOST_NO_CXX11_VARIADIC_TEMPLATES)

find_package(catkin REQUIRED COMPONENTS std_msgs visualization_msgs sensor_msgs 
image_transport image_geometry rospy roscpp opencv2 sm_kinematics pcl_ros rosbag 
ptam_com)

include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
    DEPENDS opencv2
    CATKIN_DEPENDS 
        std_msgs 
        visualization_msgs 
        sensor_msgs image_transport 
        image_geometry 
        rospy 
        roscpp 
        sm_kinematics 
        pcl_ros 
        rosbag
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
    LIBRARIES 
)

add_executable(msf_eval_run src/msf_eval.cpp)
target_link_libraries(msf_eval ${catkin_LIBRARIES})

