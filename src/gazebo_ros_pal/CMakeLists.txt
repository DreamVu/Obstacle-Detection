cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_pal)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  message_generation
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  cv_bridge
  polled_camera
  diagnostic_updater
  camera_info_manager
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
	include

  LIBRARIES
	gazebo_ros_pal_openni_kinect

  CATKIN_DEPENDS
  message_runtime
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  camera_info_manager
  std_msgs
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

#add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_library(gazebo_ros_pal_openni_kinect src/gazebo_ros_openni_kinect.cpp)
add_dependencies(gazebo_ros_pal_openni_kinect ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_pal_openni_kinect gazebo_ros_camera_utils DepthCameraPlugin ${catkin_LIBRARIES})
