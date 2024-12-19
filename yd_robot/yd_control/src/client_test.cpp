#include <std_srvs/Empty.h>
#include <boost/thread.hpp>
#include <base_control.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <yd_msgs/Pose_Task.h>
#include <ros/ros.h>
#include <iostream>
#include <boost/bind.hpp>
#include <tf/tf.h>
#include <yd_msgs/MoveLocalTargetAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>
#include <robot_msgs/TaskExecutionAction.h>
// --------------------路径计算 cpp------------------
// test for client 

void spinThread()
{
	ros::spin();
}
typedef actionlib::SimpleActionClient<yd_msgs::MoveLocalTargetAction> ClientLocalMove;
typedef actionlib::SimpleActionClient<robot_msgs::TaskExecutionAction> ClientTaskExecution;
int main(int argc, char** argv)
{
    // 初始化ROS节点s
	ros::init(argc, argv, "yd_control_client");
	ros::NodeHandle nh;
	// ClientLocalMove clientLm("ydrobot_controls",true);
	ClientTaskExecution clinetTe("task_execution",true);
  	boost::thread spin_thread(&spinThread);
	clinetTe.waitForServer();
	robot_msgs::TaskExecutionGoal TskEc;

	YAML::Node task_params;
	std::string  TaskCharge_ = "charge", TaskStop_ = "stop",TaskRotate_="robot_rotate";
	TskEc.task_type =TaskRotate_;
	double  angle_ = 3.2;
	double  speed_ = 0.2;
	task_params["angle"] =angle_;
	task_params["speed"] =speed_;
	std::stringstream Inputstring_;
	Inputstring_<<task_params;
	TskEc.parameters = Inputstring_.str();
	clinetTe.sendGoal(TskEc);
	
  	bool finished_before_timeout = clinetTe.waitForResult(ros::Duration(30.0));
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = clinetTe.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
  else
    ROS_INFO("Action did not finish before the time out.");

   ros::shutdown();
   spin_thread.join();

	return 0;
};

