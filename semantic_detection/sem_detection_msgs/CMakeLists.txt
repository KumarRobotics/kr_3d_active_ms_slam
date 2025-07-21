cmake_minimum_required(VERSION 3.0.2)
project(sem_detection_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
)


add_message_files(DIRECTORY msgs FILES
  syncPcOdom.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs nav_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs sensor_msgs nav_msgs
)