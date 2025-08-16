cmake_minimum_required(VERSION 3.8)
project(rf_main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rf_robot_msgs REQUIRED)
find_package(elog REQUIRED)
find_package(rf_global_map REQUIRED)
find_package(rf_local_map REQUIRED)
find_package(rf_global_planner REQUIRED)
find_package(rf_map_manager REQUIRED)
find_package(rf_scheduler_node REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(${PROJECT_NAME}
  src/main.cc
)

ament_target_dependencies(${PROJECT_NAME}
  rf_robot_msgs
  elog
  rf_global_map
  rf_local_map
  rf_global_planner
  rf_map_manager
  rf_scheduler_node
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
