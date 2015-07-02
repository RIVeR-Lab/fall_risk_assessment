cmake_minimum_required(VERSION 2.8.3)

project(kinect_client)

# VERY IMPORTANT TO ADD DEPENDENCIES HERE ...
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf rosconsole visualization_msgs rostime pcl_ros image_transport cv_bridge)


# ... AND HERE
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs tf rosconsole visualization_msgs rostime pcl_ros image_transport opencv cv_bridge
  )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(skeleton_client src/skeleton_client.cpp src/TCP.cpp)
target_link_libraries(skeleton_client ${catkin_LIBRARIES})

add_executable(pcl_client src/pcl_client.cpp src/TCP.cpp src/lz4.c)
target_link_libraries(pcl_client ${catkin_LIBRARIES})

add_executable(rgbd_client src/rgbd_client.cpp src/TCP.cpp src/lz4.c)
target_link_libraries(rgbd_client ${catkin_LIBRARIES})

add_executable(marker_test src/marker_test.cpp)
target_link_libraries(marker_test ${catkin_LIBRARIES})

add_executable(tf_publisher src/tf_publisher.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})

add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries(pcl_test ${catkin_LIBRARIES})

add_executable(hand_tf_converter src/hand_tf_converter.cpp)
target_link_libraries(hand_tf_converter ${catkin_LIBRARIES})

add_executable(joints src/joints.cpp src/TCP.cpp)
target_link_libraries(joints ${catkin_LIBRARIES})
