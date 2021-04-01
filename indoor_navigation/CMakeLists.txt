cmake_minimum_required(VERSION 2.8.3)
project(indoor_navigation)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  rospy
  nav_msgs
  tf
  message_generation 
  std_msgs 
  geometry_msgs
)

add_message_files(
  FILES
  soc.msg
  ctrmode.msg
  panangle.msg
  panctrmode.msg
  status.msg
  pose.msg
  path.msg
  # Message2.msg
  panimu.msg
  vision.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbot_navigation
#  CATKIN_DEPENDS geometry_msgs move_base_msgs roscpp rospy
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(odom_publisher_from_matcher src/odom_publisher_from_matcher.cpp)
target_link_libraries(odom_publisher_from_matcher ${catkin_LIBRARIES} )
add_dependencies(odom_publisher_from_matcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



add_executable(test_cmdvel_trans src/test_cmdvel_trans.cpp)
target_link_libraries(test_cmdvel_trans ${catkin_LIBRARIES} )
add_dependencies(test_cmdvel_trans ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



add_executable(imu_publisher_from_imudata src/imu_publisher_from_imudata.cpp)
target_link_libraries(imu_publisher_from_imudata ${catkin_LIBRARIES} )
add_dependencies(imu_publisher_from_imudata ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

