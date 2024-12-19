#include <base_control.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <yd_msgs/Pose_Task.h>
#include <ros/ros.h>
#include <iostream>
#include <boost/bind.hpp>
#include <tf/tf.h>
#include <yd_msgs/MoveLocalTargetAction.h>
#include <robot_msgs/Call_TaskAction.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <boost/thread.hpp>
#include <std_msgs/Bool.h>
#include <yaml-cpp/yaml.h>

/*-------------------------------------
** @name:              
** @brief:           暂时不需要抢占  bug  多客户堵塞问题  
** @param:             增action 接口
                         动作库名称      ydrobot_controls 
                         接口数据类型    yd_msgs/action/MoveLocalTarget.action
** @date:              2024-07-19
** @version:           V0.1
---------------------------------------*/

using namespace robot_msgs;
namespace basectl
{          
// ----------------------control 文件 action ------------------
void MoveLocal::Timepub(const ros::TimerEvent &_event)
{
  static  ros::Time old_time_;
  ros::Time now_time=ros::Time::now();
  // ROS_INFO(" the delta time is [%f]",(now_time-old_time_).toSec());
      if(as_.isActive())
      {
          ROS_INFO("active");
      }else 
      {
          ROS_INFO("server is waitting");
      }

  old_time_ = now_time;
  //  as_.publishFeedback(feedback_local);
}
 /*-------------------------------------
 ** @name:              
 ** @brief:             主回掉函数入口
 ** @param:              预留问题 base—control加一个 终止服务
                                     feedback 时间问题
 ** @date:              2024-07-18
 ** @version:           V0.0
 ---------------------------------------*/
void MoveLocal::executeCB(const yd_msgs::MoveLocalTargetGoalConstPtr &goal)
{
    bool SuccFlag=true;
    ros::Rate reply_(10);
    ros::Time  OpenTime=ros::Time::now();
    ros::Time  RunTime =ros::Time::now();
    double DeltaTime=(RunTime-OpenTime).toSec();
    ExpectPose.request.PoseSend = goal->PoseSend;
    ExpectPose.request.Speed = goal->Speed;
    ExpectPose.request.Id = 0;
    if((goal->PoseSend.x!=0)||(goal->PoseSend.y!=0)||(goal->PoseSend.theta!=0))
    {
        SuccFlag =false;
        AgvControl.call(ExpectPose);   
    }
    else
    {
        result_local.pose_state = true;
        result_local.PoseBack = feedback_local.PoseNow;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_local);
        return ;
    }
    DeltaTime=DeltaTime+0.00001;
    while(DeltaTime<60) //超时退出 服务器运作超时退出 重新检查进栈 定时器检测 多次检测异常 抛出异常
    {

        DeltaTime = (ros::Time::now()-RunTime).toSec();
        // ROS_INFO("wait time is [%f],status[%d]",DeltaTime,RobotStaus);
        if((RobotStaus==true)&&(DeltaTime>1.0)){SuccFlag=true; break;}
        feedback_local.workstate = RobotStaus==true? true: false;
        feedback_local.targetstate = RobotStaus==true? true:false;
        feedback_local.PoseNow=AmclNowPose;
        as_.publishFeedback(feedback_local);
        reply_.sleep();  
    }

    DeltaTime =0.0;
    if(SuccFlag==true)
    {
        result_local.pose_state = true;
        result_local.PoseBack = feedback_local.PoseNow;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_local);
        return ;       
    }
    else
    {
        result_local.pose_state=false;
        result_local.PoseBack = AmclNowPose;
        as_.setAborted(result_local);
        return ;
    }
}
void MoveLocal::goalCB(void)
{       
    _CurrentData.Speed = as_.acceptNewGoal()->Speed;
    _CurrentData.PoseSend = as_.acceptNewGoal()->PoseSend;
    _CurrentData.Id  = as_.acceptNewGoal()->Id;
    ROS_INFO("get new data");
}
void MoveLocal::GetControlStaus(const std_msgs::Bool  &Staus)
{
    if (!as_.isActive())
        return;
    if(Staus.data) {
     RobotStaus = true; }
    else{
     RobotStaus=false;}
}
void  MoveLocal::Getamcl(const geometry_msgs::PoseWithCovarianceStamped &Amcl_Pose)
    {
            tf::Matrix3x3 AmclPoseMatrix;                      
            geometry_msgs::Quaternion AmclPoseReceive=Amcl_Pose.pose.pose.orientation;
            tf::Quaternion quat_tf;
            tf::quaternionMsgToTF(AmclPoseReceive,quat_tf);
            double testyaw = tf::getYaw(quat_tf);
            AmclPoseMatrix.setRotation(quat_tf); 
            tfScalar AmclPoseYaw,AmclPosePitch,AmclPoseRoll;
            AmclPoseMatrix.getEulerYPR(AmclPoseYaw,AmclPosePitch,AmclPoseRoll);
            // AmclNowPose.header.stamp=ros::Time::now();     
            AmclNowPose.theta = AmclPoseYaw;
            AmclNowPose.x= Amcl_Pose.pose.pose.position.x;
            AmclNowPose.y = Amcl_Pose.pose.pose.position.y;
    }
//----------------------------------------------------------
void spinThread(){ ros::spin();}
//----------------------------------服务类 rm调用接口-----------------------------
void RmClientNode::DoAgvTask(std::string  task_type)
{
    if(task_type!=TaskCharge_)
    {
        YAML::Node Task_Params;
        std::stringstream DataTrans;
        //--------------other yaml
    }

    SendTask_.task_type=task_type;
    // robot_msgs::Call_TaskGoal  sendTask;
    rmc_.sendGoal(SendTask_,
               boost::bind(&RmClientNode::DoAgvTaskDo,this,_1,_2),
               boost::bind(&RmClientNode::DoAgvTaskAc,this),
               boost::bind(&RmClientNode::DoAgvTaskFd,this,_1));  
               
}
void RmClientNode::DoAgvTaskDo(const actionlib::SimpleClientGoalState &state,         //完成任务调用
                          const  robot_msgs::TaskExecutionResultConstPtr &result)
 {
      ROS_INFO("Finished state  is [%s]", state.toString().c_str());
      if(result->success==true)
      {//启动下一个任务检查队列顺序 执行下一个
         ROS_INFO("read _ next task");
      } else
      {
         ROS_INFO("succed is false'");
      }
 }
 void RmClientNode::DoAgvTaskAc() //调用任务执行时ing 此处应该调用服务
 {
    ROS_INFO("ing  using rm");  //clearing map 启动规划
    if(rmc_.waitForResult(ros::Duration(3.0))==true){}
    else
    {
         ROS_INFO("ing  using rm");  //clearing map
    }    
 }
void RmClientNode::DoAgvTaskFd(const robot_msgs::TaskExecutionFeedbackConstPtr  &feedback){}



}