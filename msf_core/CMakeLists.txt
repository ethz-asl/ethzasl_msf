cmake_minimum_required(VERSION 2.8.3)
project(msf_core)



# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs dynamic_reconfigure msf_timing tf)
find_package(Eigen REQUIRED)

include_directories(include ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -Wno-unused-parameter")

# Enable timing the core
add_definitions (-DENABLE_MSF_TIMING)

generate_dynamic_reconfigure_options(cfg/MSF_Core.cfg)

catkin_package(
    DEPENDS eigen
    CATKIN_DEPENDS roscpp sensor_msgs dynamic_reconfigure msf_timing tf
    INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
    LIBRARIES msf_core similaritytransform
    CFG_EXTRAS export_flags.cmake
)

add_library(msf_core src/lib/msf_tools.cc src/lib/falsecolor.cc src/lib/gps_conversion.cc)
target_link_libraries(msf_core ${catkin_LIBRARIES})

add_library(similaritytransform src/similaritytransform.cc)
target_link_libraries(similaritytransform ${catkin_LIBRARIES})

add_executable(test_similaritytransform src/test/test_similaritytransform.cc)
target_link_libraries(test_similaritytransform similaritytransform)

# in case anything here depends on the dynreconf file
# add_dependencies(example_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
