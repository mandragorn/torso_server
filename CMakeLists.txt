cmake_minimum_required(VERSION 2.8.3)
project(torso_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs control_msgs roscpp sensor_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(torso_server src/torso_server.cpp)
add_executable(torso_server_test_client src/test_client.cpp)

target_link_libraries(torso_server
  ${catkin_LIBRARIES}
)

target_link_libraries(torso_server_test_client
  ${catkin_LIBRARIES}
)

add_dependencies(torso_server ${catkin_EXPORTED_TARGETS})
add_dependencies(torso_server_test_client ${catkin_EXPORTED_TARGETS})
