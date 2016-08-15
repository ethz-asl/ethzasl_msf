cmake_minimum_required(VERSION 2.8.3)
project(msf_timing)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#build with boost 1.6x
add_definitions(-DBOOST_NO_CXX11_VARIADIC_TEMPLATES)

include_directories(include)

find_package(catkin REQUIRED COMPONENTS roscpp)

if (CMAKE_COMPILER_IS_GNUCC)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

catkin_package(
    DEPENDS  # TODO
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CFG_EXTRAS export_flags.cmake
)

add_library(${PROJECT_NAME} src/Timer.cc)

catkin_add_gtest(${PROJECT_NAME}_tests src/test/testMSFTiming.cc)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME} pthread)


