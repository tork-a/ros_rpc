cmake_minimum_required(VERSION 2.8.3)
project(ros_rpc)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_rpc
#  CATKIN_DEPENDS rpc_msgs
#  DEPENDS system_lib
)

install(TARGETS python_module_library
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_rpc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

if (CATKIN_ENABLE_TESTING)
  ## Add folders to be run by python nosetests
  # catkin_add_nosetests(test)
  add_rostest(test/rpc.test)
endif()
