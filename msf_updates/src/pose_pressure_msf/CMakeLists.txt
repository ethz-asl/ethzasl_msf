add_executable(pose_pressure_sensor main.cpp)
#rosbuild_add_compile_flags(pose_pressure_sensor)

target_link_libraries(pose_pressure_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(pose_pressure_sensor ${PROJECT_NAME}_gencfg})
