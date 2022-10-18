add_executable(velocity_velocity_sensor main.cpp)

# For some reason this is needed otherwise we get a weird compilation issue
target_link_libraries(velocity_velocity_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(velocity_velocity_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
