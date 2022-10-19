add_executable(flow3_position_pose_sensor main.cpp)

# For some reason this is needed otherwise we get a weird compilation issue
target_link_libraries(flow3_position_pose_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(flow3_position_pose_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
