add_executable(spherical_position_sensor main.cpp)
target_link_libraries(spherical_position_sensor ${catkin_LIBRARIES})

add_dependencies(spherical_position_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
