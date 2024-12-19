#ifndef __BASE_CONTROl_H_
#define __BASE_CONTROL_H_
#include "header_file.h"

using namespace actionlib;
using namespace std;

namespace basectl
{
	#define ABS(x)      ((x)>0?(x):-(x))
	#define limit(a,b,c)   (b>a?(b>c?c:b):a)    // b 输入  限制在 a c
	#define PI 3.141592653589793238462643383279502884						//PI
	#define RAD 3.1415926/180.0 				//Raidus  1/57.3
	#define  RaidusWheel 0.20
	#define ERRORRAD		0.02			 //精度 太小会死循环 检测不到任务结束
// bool false is theta task  bool ture is length task

typedef struct {
                        float   Speed;
                        geometry_msgs::Pose2D  PoseSend;
                        uint	  Id;
                    }   ClientData;                          //对应速度 不用msg通讯
typedef actionlib::SimpleActionClient<robot_msgs::TaskExecutionAction> ClientTaskExecution;
typedef struct 
{ 
	int ID =0; 
	float  theta, length;//任务循迹数据
	int  rotation;  //1 rotation  0 line
	int state;   // 没完成
}TaskPoint;
/**
*  
* @param theta which client set    
* @param Clientspeed the speed which  client set 
* @param ExpectStack every point which we want set 
* @
eturn  none     
*/
typedef struct
{	
	float theta,distance; //期望角速度
	std_msgs::Float64 Clientspeed;
	vector<TaskPoint> ExpectStack;						//角速度stack 队列！！
}Agv_Control;
typedef struct { std_msgs::Float64 liner_x; std_msgs::Float64 angular_z; }Ctwist;
class  Robot_Controller
{
	protected:
			ros::NodeHandle nh;
			std::string action_name_;
			yd_msgs::MoveLocalTargetActionFeedback  feedback_slm_;
			yd_msgs::MoveLocalTargetActionResult    result_slm_;
			yd_msgs::MoveLocalTargetActionGoal      goal_slm_;

	public:
			Robot_Controller(){InitNodeHandle();}
			~Robot_Controller();
			void 	Task();
			bool    Staus(void);		//控制器主要开关口
			void    clearmap()
			{
				      clear_map.request={};
                      clear_client_base.call(clear_map);
			}
			bool   Dynatic_Controller(const geometry_msgs::Pose2D &ctrl, float clientspeed);
	private:
	 //配置速度  位置
	 void	Speed_Set(const geometry_msgs::Twist &setwist); 			//set speed 
	 geometry_msgs::Pose2D 	Pose_Set(const geometry_msgs::Pose2D &SetPose); //set pose of the target
	 vector<TaskPoint> PathDiv(geometry_msgs::Pose2D &startP ,geometry_msgs::Pose2D &endP,std_msgs::Float64 &speed,int length);
     TaskPoint PathLine(geometry_msgs::Pose2D &startP,geometry_msgs::Pose2D &endP); //get the taks point of the line
	 void     executeCB(const yd_msgs::MoveLocalTargetActionGoalConstPtr &goal){}
	 void 	  InitNodeHandle(void);
	 void     PoseEkf_Get(const geometry_msgs::PoseWithCovarianceStamped &ekf_pose);					//get the pose of the ekf odom
	 bool 	  ServerTask(yd_msgs::Pose_Task::Request &ClientReq, yd_msgs::Pose_Task::Response &Serveres);   //get server Task;
	 void     PoseOdom_Get(const nav_msgs::Odometry &odom_pose);  				
	 void     PublishCmdRtime(Ctwist &ctwist);			//publish  realtime
	 float 	  DistancePoint(geometry_msgs::Pose2D& a,geometry_msgs::Pose2D& b);
	 float 	  ThetaPoint(geometry_msgs::Pose2D& a,geometry_msgs::Pose2D& b);
     void     BackToCharge(const sensor_msgs::BatteryState &Power_);
	 TaskPoint Start_Point,End_Point;
	 bool *ServiceIndex; //set up adderss
	 yd_msgs::Pose_Task::Response *YdmsgsIndex; //
	 void  PathTask(void);
	 void TimetaksPub(const ros::TimerEvent &_event);
	
	 geometry_msgs::Twist Twist_pub,TwistBase;
	 Agv_Control  Static_Control,Dynatic_Control;  //static control
	 ros::Timer TimeSpeed_;


	 vector<TaskPoint> ExpectStackLine;					//线速度stack
 	 ros::ServiceServer  Pose_server ;
 	 std_srvs::Empty  clear_map;

     ros::ServiceClient  clear_client_base;

	 ros::Time  _Now,
	 		    _Last_Time,
	 			_Old_Time;
	 ros::Subscriber Ekf_PoseSub,						//ekf订阅
					 Power_Sub,							//电源订阅
					 Odom_PoseSub;						//原始odom订阅

	 yd_msgs::Pose_Task::Request rolletest,
	 							pose_request;			//服务器数据
	 static yd_msgs::Pose_Task::Request RmSetPose; 		//服务器设定位置 静态坐标
	 ros::Publisher Twistpublisher,					    //速度发布
	 				path_pb,						    //路径发布
					pose_publisher,						//位置发布
					BackToChargePublisher,				//回充发布
					ControlSate;						//机器人移动状态
					
	 geometry_msgs::Pose2D  TaskOld_pose,				//任务开始坐标
	 						current_pose, 				//当前实际坐标
	 						target_pose,				//期望坐标
							error_pose,					//误差坐标
							ActionOldPose_,			//目标坐标ekf 为准
							CurrentPoseEkf,				//全局ekf坐标 
							CurrentPoseOdom;			//全局odom坐标
	 Ctwist  cureent_twist,
	 		 old_twist,
			 new_twist;
	 vector<geometry_msgs::Pose2D> Ekf_Pose;    		  //old data vector
	 vector<geometry_msgs::Pose2D> PoseVectorEkf;

	 vector<Ctwist>  twist_stack;   	 
	 vector<ros::Time> _TimeFollow;

};

//action movelocal
class MoveLocal{

    protected:
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<yd_msgs::MoveLocalTargetAction> as_;
            std::string action_name_;
            float sum_, sum_sq_;
            // geometry_msgs::PoseWithCovarianceStamped  AmclNowPose;  
            geometry_msgs::Pose2D  AmclNowPose;  
            yd_msgs::MoveLocalTargetFeedback  feedback_local;
            yd_msgs::MoveLocalTargetResult    result_local;
            yd_msgs::MoveLocalTargetGoal      goal_local;
            ros::Subscriber amcl_pose,\
                            goal_result,\
                            twist_result;
            ros::ServiceClient AgvControl;
            yd_msgs::Pose_Task ExpectPose;

    public:
            MoveLocal(std::string name) : 
            // as_(nh_, name, boost::bind(&MoveLocal::executeCB, this, _1), false),
           as_(nh_, name,boost::bind(&MoveLocal::executeCB,this,_1),false),action_name_(name)
    {
        // as_.registerGoalCallback(boost::bind(&MoveLocal::goalCB, this));  //正常回掉函数接口
        // as_.registerPreemptCallback(boost::bind(&MoveLocal::preemptCB, this));      //抢占回掉接口
        amcl_pose =nh_.subscribe("/amcl_pose",10,&MoveLocal::Getamcl,this);    //as callback
        AgvControl = nh_.serviceClient<yd_msgs::Pose_Task>("/Rm_TargetPose");
        twist_result = nh_.subscribe("/ControState",10,&MoveLocal::GetControlStaus,this);
        TimeTest_ = nh_.createTimer(ros::Duration(1), &MoveLocal::Timepub,this,false,true);  
        ROS_INFO("open server");
        as_.start();
    }
         ~MoveLocal(void){}
        ClientData _GetData;
        geometry_msgs::Pose2D  UpdateActionPose(void )
        {
                return as_.acceptNewGoal()->PoseSend;
        }
        float  UpdateActionSpeed(void)
        {
                return as_.acceptNewGoal()->Speed;
        }
    private:

    	ros::Timer TimeTest_;
        ClientData _CurrentData, _NewData,_OldData;
        bool RobotStaus;
        void Timepub(const ros::TimerEvent &_event);
        void executeCB(const yd_msgs::MoveLocalTargetGoalConstPtr &goal);
        void  goalCB(void);
        void  preemptCB()                                                           //抢占回掉接口   
        {               
             ROS_INFO("get new data");
                // as_.setPreempted();
        }
        void   GetControlStaus(const std_msgs::Bool  &Staus);                      //控制器判断 不是速度判断
        void   GetGoalState(const move_base_msgs::MoveBaseActionResult &MoveGoalResult){}
        void   Getamcl(const geometry_msgs::PoseWithCovarianceStamped &Amcl_Pose);

};
//rm client
class   RmClientNode
{
 public:
         RmClientNode(std::string name):
         rmc_(name,true) //定义一个rm 任务控制器 !
         {
             ROS_INFO("open client");
             WaitForServer(5.0);             //5s等待报错
         }
         bool WaitForServer(void)  {
             return rmc_.waitForServer(ros::Duration(3.0));
         }
         void WaitForServer(float errortime){
             rmc_.waitForServer(ros::Duration(errortime));
         }
         void DoAgvTask(std::string  task_type );

         void DoAgvTaskDo(const actionlib::SimpleClientGoalState &state,         //完成任务调用
                          const  robot_msgs::TaskExecutionResultConstPtr &result);
         void DoAgvTaskAc();        //调用任务执行时
         void DoAgvTaskFd(const robot_msgs::TaskExecutionFeedbackConstPtr  &feedback);
         robot_msgs::TaskExecutionResult  GetRmResult(void) {
                return  *(rmc_.getResult());    }
         bool isConnected(void){  return  rmc_.isServerConnected();}
         void  SpinThread_(void){ros::spin();}
          ~RmClientNode(){}
 protected:
          std::string   TaskCharge_ = "charge", 
                        TaskStop_ = "stop",
                        TaskRotate_="robot_rotate";
          ClientTaskExecution rmc_;
          robot_msgs::TaskExecutionGoal    SendTask_;
          robot_msgs::TaskExecutionFeedback Taskfeedback_; // 状态反馈
          robot_msgs::TaskExecutionResult   Taskresult_;   //
 private:
            boost::recursive_mutex armclient_mutex;
            boost::condition_variable_any armclient_condition_variable;

            boost::thread* armclient_thread_;

};

}
#endif
