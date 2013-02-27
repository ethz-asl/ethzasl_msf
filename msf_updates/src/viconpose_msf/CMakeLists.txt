rosbuild_add_executable(viconpose_sensor main.cpp)
rosbuild_add_compile_flags(viconpose_sensor "-O2")

target_link_libraries(viconpose_sensor pose_distorter)
