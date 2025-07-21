cmake_minimum_required(VERSION 3.16)
project(sloam_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib_msgs
)

add_message_files(DIRECTORY msg FILES
    ROSCylinder.msg
    ROSGround.msg
    ROSScan.msg
    ROSSweep.msg
    ROSSubMap.msg
    ROSObservation.msg
    ObservationPair.msg
    LoopClosure.msg
    ROSRangeBearing.msg
    ROSRangeBearingSyncOdom.msg
    SemanticLoopClosure.msg
    ROSSyncOdom.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  ActiveLoopClosure.action
  DetectLoopClosure.action
)

add_service_files(
  FILES
  EvaluateLoopClosure.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs actionlib_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs nav_msgs actionlib_msgs
)