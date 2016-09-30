add_executable(multi_pose_sensor main.cpp)

target_link_libraries(multi_pose_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(multi_pose_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
