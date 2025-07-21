cmake_minimum_required(VERSION 2.8.3)
project(cop_lib)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")


find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cop_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
)

add_library( cop_lib
    src/cop.cpp
)
target_link_libraries(cop_lib
    ${catkin_LIBRARIES}
) 

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)