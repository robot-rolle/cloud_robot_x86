cmake_minimum_required(VERSION 3.0.2)
project(using_markers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)


catkin_package(

)

include_directories(include/${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(basic_block  src/basic_block.cpp src/path_plan_rrt.cpp src/path_plan_rrt_star.cpp)

target_link_libraries(basic_block ${catkin_LIBRARIES})
