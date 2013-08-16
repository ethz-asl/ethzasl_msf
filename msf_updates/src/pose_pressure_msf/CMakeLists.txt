add_executable(pose_pressure_sensor main.cpp)

target_link_libraries(pose_pressure_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(pose_pressure_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
