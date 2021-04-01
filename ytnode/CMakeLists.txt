cmake_minimum_required(VERSION 2.8.3)
project(ytnode)

set (CMAKE_CXX_FLAGS "-std=c++11 -O3") 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  tf
  image_transport
  nav_msgs
  message_generation
  message_filters
)


add_message_files(  
	FILES 
        panangle.msg
)



generate_messages (DEPENDENCIES 
std_msgs )

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_executable(ytnode
  src/ytnode.cpp
)
add_dependencies(ytnode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ytnode
  ${catkin_LIBRARIES}
)



