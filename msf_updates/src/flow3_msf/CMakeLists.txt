add_executable(flow3_sensor main.cpp)

# For some reason this is needed otherwise we get a weird compilation issue
target_link_libraries(flow3_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(flow3_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
