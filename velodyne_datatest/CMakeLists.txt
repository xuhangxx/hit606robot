cmake_minimum_required(VERSION 2.8.3)
project(velodyne_datatest)

set (CMAKE_CXX_FLAGS "-std=c++11 -O3") 

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  serial
)
set(Boost_LIBRARY_DIR /usr/lib/x86_64-linux-gnu/)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(FILES
  velodyne_obstacle.msg
)


generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
 sensor_msgs
 DEPENDS EIGEN3 PCL OpenCV
)



include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIR} 
	${PCL_INCLUDE_DIRS})
  

add_library(${PROJECT_NAME}
  src/velodyne_datatest.cpp
)



add_executable(${PROJECT_NAME}_node
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${PCL_LIBRARIES} ${OpenCV_LIBS}
)



