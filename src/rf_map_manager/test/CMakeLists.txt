add_executable(test_map_manager_node
  test_map_manager_node.cc
)

target_link_libraries(test_map_manager_node PUBLIC
  ${PROJECT_NAME}
)

install(
  TARGETS test_map_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)