cmake_minimum_required(VERSION 2.8.3)
project(decision_node)

set( CMAKE_CXX_FLAGS "-std=c++11 -O3")
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  tf
  nav_msgs
  geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES decision_node
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)



add_library(${PROJECT_NAME}
  src/decision_node.cpp
)


add_executable(${PROJECT_NAME}_node
  src/node.cpp
)

#add_dependencies(node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)



add_executable(cpp_decision
  src/cpp_decision.cpp
)

#add_dependencies(node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cpp_decision
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#add_executable(decision
 # src/decision.cpp
#)

#add_dependencies(node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(decision
 # ${catkin_LIBRARIES}
 # ${PROJECT_NAME}
#)



