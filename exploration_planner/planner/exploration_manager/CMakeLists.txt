cmake_minimum_required(VERSION 2.8.3)
project(exploration_manager)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  kr_mav_msgs
  kr_tracker_msgs
  plan_env
  path_searching
  traj_utils
  active_perception
  plan_manage
  cv_bridge
  decomp_ros_utils
  exploration_msgs
  actionlib
  actionlib_msgs
  sloam_msgs
  gcopter
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES exploration_manager
 CATKIN_DEPENDS 
  plan_env 
  path_searching 
  active_perception 
  traj_utils 
  plan_manage 
  exploration_msgs
  actionlib
  actionlib_msgs
  std_msgs
  sloam_msgs
  gcopter
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(exploration_manager src/ms_exploration_manager.cpp)
add_library(exploration_fsm src/ms_exploration_fsm.cpp src/ms_exploration_manager.cpp)
add_library(global_plan_server src/global_plan_server.cpp)

add_executable(exploration_node
  src/exploration_node.cpp 
  )

target_link_libraries(exploration_node
  exploration_manager
  exploration_fsm
  ${catkin_LIBRARIES}
  # -ldw
)

add_executable(global_plan_server_node
  src/global_plan_server_node.cpp
  )

target_link_libraries(global_plan_server_node 
  global_plan_server
  ${catkin_LIBRARIES}
)

add_dependencies(exploration_node  ${catkin_EXPORTED_TARGETS})
add_dependencies(global_plan_server_node  ${catkin_EXPORTED_TARGETS})