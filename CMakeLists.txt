cmake_minimum_required(VERSION 2.8.3)
project(test_acm_moveit)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  pr2_mover_utils
  roscpp
  std_msgs
  tf
  trajectory_msgs
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES test_acm_moveit
  CATKIN_DEPENDS moveit_ros_planning_interface pr2_mover_utils roscpp std_msgs tf trajectory_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_acm_moveit src/test_acm_moveit.cpp)
target_link_libraries(test_acm_moveit ${catkin_LIBRARIES}  )