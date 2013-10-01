add_executable(position_sensor main.cpp)

target_link_libraries(position_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(position_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
