cmake_minimum_required(VERSION 3.0.2)
project(yd_agv_driver)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  nav_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  serial
  std_msgs
  tf
)

catkin_package(

)

include_directories(include/${PROJECT_NAME})
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(yd_robot_node src/yd_robot.cpp src/yd_robot_data.cpp)
target_link_libraries(yd_robot_node ${catkin_LIBRARIES})