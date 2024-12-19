cmake_minimum_required(VERSION 3.0.2)
project(yd_global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  costmap_2d
  nav_core
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  global_planner
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES yd_global_planner
 CATKIN_DEPENDS angles global_planner base_local_planner costmap_2d nav_core pluginlib roscpp tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

add_library(yd_global_planner
             src/yd_planner.cpp 
             src/mtrand.cpp
             src/Grid_PathMap.cpp
             src/yd_math.cpp
             src/yd_planner_core.cpp
             )
add_dependencies(yd_global_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(yd_global_planner
    ${catkin_LIBRARIES}
    )

    install(TARGETS yd_global_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN ".svn" EXCLUDE
)

install(FILES ydg_plugin.xml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


