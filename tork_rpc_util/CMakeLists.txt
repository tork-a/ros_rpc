cmake_minimum_required(VERSION 2.8.3)
project(tork_rpc_util)

find_package(catkin REQUIRED message_generation roslint std_msgs)

catkin_python_setup()

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tork_rpc_util
  CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)

# TODO Enable tests.
#if (CATKIN_ENABLE_TESTING)
#  add_rostest(test/rpc.test)
#endif()

roslint_python(src/tork_rpc_util/rpc_servers_handler.py)
