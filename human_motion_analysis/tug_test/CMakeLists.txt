cmake_minimum_required(VERSION 2.8.3)
project(tug_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  openni_tracker
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_logger src/tf_logger.cpp)
target_link_libraries(tf_logger yaml-cpp
   ${catkin_LIBRARIES}
 )

add_executable(visualize src/visualize.cpp)
target_link_libraries(visualize
   ${catkin_LIBRARIES}
 )
