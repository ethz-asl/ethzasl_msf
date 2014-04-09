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
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} src/msf_simulator.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(test_msf_simulator src/test/main.cc 
                                    src/test/test_simulated_trajectory.cc)
target_link_libraries(test_msf_simulator ${PROJECT_NAME})


