cmake_minimum_required(VERSION 3.8)
project(rf_map_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rf_robot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(elog REQUIRED)
find_package(rf_util REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cartographer REQUIRED)

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

add_library(${PROJECT_NAME} STATIC
  src/map_builder_bridge.cc
  src/msg_conversion.cc
  src/sensor_bridge.cc
  src/map_builder_node.cc
  src/config_options.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PCL_LIBRARIES}
  cartographer)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include)

# 混合编译带有 package.xml 和没有 package.xml 包的情况
# 需要手动调用 target_link_libraries 来找到没有 package.xml 的包
# 直接调用 ament_target_dependencies 会找不到
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rf_robot_msgs
  elog
  rf_util
  geometry_msgs
  sensor_msgs
  pcl_conversions
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rf_robot_msgs
  elog
  rf_util
  geometry_msgs
  sensor_msgs
  pcl_conversions
  cartographer
)

ament_package()
