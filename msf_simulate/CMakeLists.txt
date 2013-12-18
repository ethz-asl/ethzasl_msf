cmake_minimum_required(VERSION 2.8.3)
project(msf_simulate)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS msf_core msf_updates)
find_package(Eigen REQUIRED)

include_directories(include ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++0x")

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS msf_core msf_updates
    INCLUDE_DIRS 
    LIBRARIES 
)

add_library(${PROJECT_NAME} src/MSFSimulator.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_tests src/test/main.cc src/test/test_simulated_trajectory.cc)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})


