cmake_minimum_required(VERSION 2.8.3)
project(msf_distort)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#build with boost 1.6x
add_definitions(-DBOOST_NO_CXX11_VARIADIC_TEMPLATES)

find_package(catkin REQUIRED COMPONENTS msf_core std_msgs visualization_msgs sensor_msgs image_transport image_geometry geometry_msgs dynamic_reconfigure)

include_directories(${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(cfg/MSF_Distort.cfg)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS msf_core std_msgs visualization_msgs sensor_msgs image_transport image_geometry geometry_msgs dynamic_reconfigure
    INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
    LIBRARIES
)

add_executable(msf_distort src/msf_distort.cc)
target_link_libraries(msf_distort ${catkin_LIBRARIES} ${boost_LIBRARIES})
add_dependencies(msf_distort ${${PROJECT_NAME}_EXPORTED_TARGETS})

