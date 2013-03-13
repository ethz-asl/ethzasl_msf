rosbuild_add_executable(position_sensor main.cpp)
rosbuild_add_compile_flags(position_sensor "-O2")

target_link_libraries(position_sensor pose_distorter)