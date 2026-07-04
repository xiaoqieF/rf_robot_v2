get_property(EIGEN_TESTS_LIST GLOBAL PROPERTY EIGEN_TESTS_LIST)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/buildtests.in")
  configure_file(buildtests.in ${CMAKE_BINARY_DIR}/buildtests.sh @ONLY)
endif()

configure_file(check.in ${CMAKE_BINARY_DIR}/check.sh COPYONLY)
configure_file(debug.in ${CMAKE_BINARY_DIR}/debug.sh COPYONLY)
configure_file(release.in ${CMAKE_BINARY_DIR}/release.sh COPYONLY)
