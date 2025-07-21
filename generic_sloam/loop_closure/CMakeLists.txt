cmake_minimum_required(VERSION 3.10)
project(loop_closure)

set(CMAKE_CXX_STANDARD 17)
# find_package(Boost REQUIRED COMPONENTS timer)
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
  tf2
  tf2_eigen
  tf2_geometry_msgs
  sloam_msgs
  actionlib
  actionlib_msgs
  sloam_msgs
  )

find_package(GTSAM REQUIRED)


find_package (Eigen3 3.3 REQUIRED NO_MODULE)


catkin_package(
   INCLUDE_DIRS include src
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
   tf2
   tf2_eigen
   tf2_geometry_msgs
   actionlib
   actionlib_msgs
   sloam_msgs
   DEPENDS)

add_library(${PROJECT_NAME} src/loopClosureServer.cpp src/measurement_processing.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC src include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} PRIVATE Eigen3::Eigen)

add_executable(loop_closure_server_node src/loopClosureServerNode.cpp)
target_link_libraries(loop_closure_server_node PUBLIC ${PROJECT_NAME} PRIVATE Eigen3::Eigen gtsam)
add_dependencies(loop_closure_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
