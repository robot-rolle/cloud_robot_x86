#include<base_control.h>
#include <robot_msgs/Call_TaskAction.h>
using namespace basectl;

//----------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::Time::init();
	  ros::init(argc, argv, "ydrobot_controls");
    ros::Rate loop_rate(10);
    basectl::MoveLocal Ydctrl(ros::this_node::getName());
    ros::spin();
    return 1;
}