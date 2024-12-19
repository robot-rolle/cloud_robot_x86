cmake_minimum_required(VERSION 3.0.2)
project(yd_control)


find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    rospy
    robot_msgs
    std_msgs  
    message_runtime
    yd_msgs
    message_generation
    actionlib_msgs
    actionlib
    move_base_msgs
    std_srvs
    roslib
    sensor_msgs
    dynamic_reconfigure
)


generate_messages(
  DEPENDENCIES
  actionlib_msgs  
  geometry_msgs
  std_msgs
  actionlib

)

generate_dynamic_reconfigure_options(
    cfg/BaseAction.cfg
)

catkin_package(
   CATKIN_DEPENDS 
    geometry_msgs
    roscpp 
    rospy
    std_msgs 
    message_runtime
    yd_msgs
    actionlib_msgs
    actionlib
    robot_msgs
    move_base_msgs
    std_srvs
    dynamic_reconfigure
)


include_directories(include ${catkin_INCLUDE_DIRS} )
# Declare a C++ library
# add_library(${PROJECT_NAME}
#     src/base_control_node.cpp
#     src/base_control.cpp
# )

add_executable(yd_control_base 
               src/base_control_node.cpp 
               src/base_control.cpp)
target_link_libraries(yd_control_base   
        yaml-cpp ${catkin_LIBRARIES})
add_dependencies(yd_control_base ${PROJECT_NAME}_gencpp)


add_executable(yd_control_client  
               src/client_test.cpp)
target_link_libraries(yd_control_client
  yaml-cpp
  ${catkin_LIBRARIES})
add_dependencies(yd_control_client ${PROJECT_NAME}_gencpp)

# 
add_executable(ydrobot_controls  
              src/ydrobot_controls.cpp 
              src/base_control.cpp 
              src/ydrobot_controls_node.cpp
              src/base_action.cpp )
target_link_libraries(ydrobot_controls 
       yaml-cpp 
       ${catkin_LIBRARIES})
# add_dependencies(ydrobot_controls ${PROJECT_NAME}_gencpp)
add_dependencies(ydrobot_controls 
                ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                ${catkin_EXPORTED_TARGETS} 
                ${PROJECT_NAME}_gencpp)



#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

#------------------
