cmake_minimum_required(VERSION 2.8.3)
project(ros_rpc)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_rpc
#  CATKIN_DEPENDS rpc_msgs
#  DEPENDS system_lib
)

# TODO Enable tests.
#if (CATKIN_ENABLE_TESTING)
#  add_rostest(test/rpc.test)
#endif()
