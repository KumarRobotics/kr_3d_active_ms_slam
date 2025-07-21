cmake_minimum_required(VERSION 3.16)
project(cloud_undistort)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_eigen
  tf2
  tf
  roscpp
)


find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

catkin_package(
   INCLUDE_DIRS
   ${Sophus_INCLUDE_DIRS}
   LIBRARIES
   ${PROJECT_NAME}
   CATKIN_DEPENDS
   pcl_ros
   pcl_conversions
   tf2_geometry_msgs
   tf2_ros
   tf2_eigen
   tf2
   tf
   roscpp
)

add_definitions(${PCL_DEFINITIONS})
add_executable(${PROJECT_NAME} src/cloud_undistort_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Sophus_INCLUDE_DIRS} ${catkin_LIBRARIES} fmt glog)
