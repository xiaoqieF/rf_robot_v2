add_executable(test_costmap
  test_costmap.cc
)

target_link_libraries(test_costmap PUBLIC
  ${PROJECT_NAME}
)

install(
  TARGETS test_costmap
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)