rosbuild_add_executable(pose_sensor main.cpp)
rosbuild_add_compile_flags(pose_sensor "-O2")

target_link_libraries(pose_sensor pose_distorter)
