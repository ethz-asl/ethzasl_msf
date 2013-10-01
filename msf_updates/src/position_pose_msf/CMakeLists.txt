add_executable(position_pose_sensor main.cpp)

target_link_libraries(position_pose_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(position_pose_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
