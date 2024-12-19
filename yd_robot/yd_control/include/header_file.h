#ifndef _HEADER_FILE_H_
#define _HEADER_FILE_H_

#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <bits/stdc++.h>
#include <yd_msgs/Pose_Task.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <yd_msgs/MoveLocalTargetAction.h>
#include <boost/bind.hpp>
#include  <std_srvs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <robot_msgs/Call_TaskAction.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/BatteryState.h>
#include <yaml-cpp/yaml.h>
#include <robot_msgs/TaskExecutionAction.h>
#include <dynamic_reconfigure/server.h>

#endif
