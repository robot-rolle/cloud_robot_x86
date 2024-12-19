#include "base_control.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "yd_msgs/Pose_Task.h"
#include <ros/ros.h>
#include <iostream>
#include <boost/bind.hpp>
#include <tf/tf.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"

using namespace std;
using namespace basectl;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"yd_control_base"); 
	Robot_Controller yd_control;
   
    ros::Rate loop_rate(10);

    while(ros::ok())
    { 
        ros::spinOnce();
        yd_control.Task();
	    loop_rate.sleep();
    }
    return 1;
}
