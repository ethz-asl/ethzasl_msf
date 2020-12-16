add_executable(pose_sensor main.cpp)

target_link_libraries(pose_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(pose_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS pose_sensor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
