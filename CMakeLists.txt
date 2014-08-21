cmake_minimum_required(VERSION 2.8.3)
project(amigo_spindle_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  amigo_actions
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs amigo_actions roscpp sensor_msgs
  DEPENDS thread
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(spindle_server src/spindle_server.cpp)
add_executable(spindle_server_test_client src/test_client.cpp)

target_link_libraries(spindle_server
  ${catkin_LIBRARIES}
)

target_link_libraries(spindle_server_test_client
  ${catkin_LIBRARIES}
)

add_dependencies(spindle_server ${catkin_EXPORTED_TARGETS})
add_dependencies(spindle_server_test_client ${catkin_EXPORTED_TARGETS})
