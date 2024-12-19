
#include "base_control.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "yd_msgs/Pose_Task.h"
#include <ros/ros.h>
#include <iostream>
#include <boost/bind.hpp>
#include <tf/tf.h>
#include <mutex>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
using namespace std;
// void basectl::SpeedUpDate(const geometry_msgs::Twist &twist_aux);
/**
 * @Date: 2024-06-28 09:33:21
 * @Input : none ,Return:none
 * @breif://主要的task  bug汇总  自转绝对角度  位置控制相对角度
 * @MethodAuthor: Rolle
 * @param :  
 * @功能 :
*/

namespace basectl
{
void Robot_Controller::Task()
{
    PathTask(); 
    static int count_time =0;
    std_msgs::Bool  ControlStausSend;
    ControlStausSend.data =Staus();
    ControlSate.publish(ControlStausSend);
    if(ControlStausSend.data)
    {       count_time++;}
    else { count_time=0;}

    if(count_time>50)
    {    count_time=0; 
        clearmap();
    }else;
}

 /**
* @param  PublishCmdRtime
* @param   
* @param   
turn   none
*/
 void Robot_Controller::PublishCmdRtime(Ctwist &twist_aux)//publish  realtime
 {
    double Deltatime;
    geometry_msgs::Twist SetTwist_;
    _Now = ros::Time::now(); 
    SetTwist_.linear.x=twist_aux.liner_x.data;
    SetTwist_.angular.z= twist_aux.angular_z.data;
    /**
    * @param  速度下发调试
    */
    Speed_Set(SetTwist_);  //速度下发
    Deltatime=(_Now - _Old_Time).toSec();
    _Old_Time=_Now;
 }

 vector<TaskPoint> Robot_Controller::PathDiv(geometry_msgs::Pose2D &startP ,geometry_msgs::Pose2D &endP,std_msgs::Float64 &speed,int length)
{
        vector<TaskPoint> BufferPoint;
        float DeltaTime=10.0;           //发布频次10次
        float Linetheta,PathLength;
        float StepSize= (speed.data*DeltaTime);
        int CountStep;
        length = CountStep;
        Linetheta=((endP.y-startP.y)/(endP.x-startP.x));                 //获取theta
        PathLength=sqrt(pow((endP.y-startP.y),2)+pow((endP.x-startP.x),2));
        CountStep=PathLength/StepSize;                      //1s  下的单位时间路长
        for(int i =0;i<CountStep;i++ )                      //分割路径节点
        {
            BufferPoint[CountStep].rotation=0;
            BufferPoint[CountStep].state=0;
            BufferPoint[CountStep].theta =0.0;
            BufferPoint[CountStep].length=StepSize;
            BufferPoint[CountStep].ID =CountStep;                   
        }
        return   BufferPoint;
}
 /**
* @param  main server task   
* @param   clientreq   is the client task serveres
* @param   服务控制器 配置move_group接口   拆分路径为局部状态 每个状态设定状态位 队列存储  srv控制调试接口
            x   |   cos theta         0       |  +   x1 = x2
            y   |   0               sin theta |  +   y1 =  y2
eturn   bool 服务器返回状态  
*/
bool   Robot_Controller::ServerTask(yd_msgs::Pose_Task::Request &ClientReq, yd_msgs::Pose_Task::Response &Serveres)
{
    float theta_origin=0.0; //开始theta
    float buffer_origin=0.0;
    TaskPoint LinePoint;
    memset(&LinePoint,0,sizeof(LinePoint));
    YdmsgsIndex =&Serveres;
	memset(&Twist_pub,0,sizeof(Twist_pub)); //初始化 0
    // 显示请求数----------------位置精度判断 取决于里程计和imu融合后数据---------
    ROS_INFO("Pose  x: %f, y: %f, theta :%f ,speed:%f", ClientReq.PoseSend.x, ClientReq.PoseSend.y, ClientReq.PoseSend.theta,ClientReq.Speed);
    if((ClientReq.PoseSend.x==0.00)&&(ClientReq.PoseSend.y==0.00)&&(ABS(ClientReq.PoseSend.theta)!=0)) 
    {        Serveres.pose_state=true;   
            memset(&Start_Point,0,sizeof(Start_Point));
            Start_Point.theta = ClientReq.PoseSend.theta;
            Start_Point.state=0;
            Start_Point.rotation= 1;
            Start_Point.length=0.0;
            Static_Control.ExpectStack.push_back(Start_Point);//存入任务点 包含任务目标和任务状态
              return true;     }
    //----------------------------------mini局部路径规划信息存储--------------------------
    else
    {   
         Serveres.pose_state = true;                                   //开始局部循迹  服务器同步
        cout<<" the state is "<<YdmsgsIndex[0]<<endl;
        if(Static_Control.ExpectStack.size()==0) 
        {
             memset(&TaskOld_pose,0,sizeof(TaskOld_pose));          
             TaskOld_pose=CurrentPoseEkf;                                  //O(odom)  原始
             target_pose.x=TaskOld_pose.x+ClientReq.PoseSend.x;
             target_pose.y=TaskOld_pose.y+ClientReq.PoseSend.y;
             target_pose.theta=TaskOld_pose.theta+ClientReq.PoseSend.theta;//此处错误     期望角度 =O(odom)+O(pos)-O(pos)+O(req);
        }
        else   
        {
            ROS_INFO("------REBUG----OLD------size is [%ld]",Static_Control.ExpectStack.size()  );
            return true;
        }
        theta_origin=atan2((ClientReq.PoseSend.y),(ClientReq.PoseSend.x)); //获取路径角度   得到弧度 
        buffer_origin = theta_origin+TaskOld_pose.theta;  //Odom+O(pos)  

        buffer_origin= buffer_origin>PI? \
                       buffer_origin-(2*PI):\
                       buffer_origin<(-1 *PI)?\
                       buffer_origin+(2*PI):\
                       buffer_origin;
        Start_Point.theta = buffer_origin;  //服务设定角度
        Start_Point.length=0.0;     
        if(buffer_origin==0.0){Start_Point.rotation=0;}  //初始化第一个任务的自转
        else    {    Start_Point.rotation=1;   }
        Start_Point.state = 0;
        Start_Point.ID = 0;
        Static_Control.distance=sqrt(pow((ClientReq.PoseSend.x-TaskOld_pose.x),2)+pow((ClientReq.PoseSend.y-TaskOld_pose.y),2));//path length 分割长度后续优化 长度判断符
        if(Static_Control.distance>5)
        {   
            ROS_INFO("This is Error Distance which over local plan");  //此处调试清空所有标志位
        }
        else;
        Static_Control.ExpectStack.push_back(Start_Point);//存入任务点 包含任务目标和任务状态
        if(ClientReq.Speed==0.0){ Static_Control.Clientspeed.data =0.1;  }else{            //默认速度配置
        Static_Control.Clientspeed.data= limit(-0.8,ClientReq.Speed,0.8);  }              //客户端限幅

        LinePoint= PathLine(TaskOld_pose,target_pose);
        Static_Control.ExpectStack.push_back(LinePoint);                                //
     //   End_Point.theta =ClientReq.PoseSend.theta-TaskOld_pose.theta;           //      结束状态 theta 最后存储 对路径过程分割 约束到最小时存储  //bug3
        End_Point.theta = target_pose.theta;
        //对角度差限制幅度在【-PI，PI】
        End_Point.theta = End_Point.theta >PI? \
                          End_Point.theta-(2*PI):\
                          End_Point.theta<(-1*PI)?\
                          End_Point.theta+(2*PI):\
                          End_Point.theta;    
        End_Point.rotation =1;
        End_Point.length= 0.0;   
        End_Point.state =false;
        Static_Control.ExpectStack.push_back(End_Point);
    }
    return true;
}

TaskPoint Robot_Controller::PathLine(geometry_msgs::Pose2D &startP,geometry_msgs::Pose2D &endP)
{
        TaskPoint LineTaskPoint;
        LineTaskPoint.rotation =0;
        LineTaskPoint.state =0;
        LineTaskPoint.length =DistancePoint(startP,endP);
        LineTaskPoint.theta =ThetaPoint(startP,endP);
        return LineTaskPoint;
}
/**  
* @param   Path Check Control
* @param    路径缓存控制
* @param 
*/
void Robot_Controller::PathTask(void)
{   
    static int Initcount=0;
    float DeltaAngle;
    if(Static_Control.ExpectStack.size()>0) //任务队中有数据还没处理
    {    
        Initcount=0;  
        if((Static_Control.ExpectStack.front().rotation==1)&&(Static_Control.ExpectStack.front().state==0)) //第一个任务是自转和第一个任务是没有完成
        {  
                            // ROS_INFO("----------line");
            new_twist.angular_z.data=Static_Control.Clientspeed.data;
            new_twist.liner_x.data= 0.0;
           //   ROS_INFO("----- delta theta is[%f],current theta is[%f] now theta is[%f],expec[%f]",\
                    (CurrentPoseEkf.theta-TaskOld_pose.theta),\
                    CurrentPoseEkf.theta,TaskOld_pose.theta,\
                    Static_Control.ExpectStack.front().theta);
            //speed 控制                                                  
             if(ABS(CurrentPoseEkf.theta-Static_Control.ExpectStack.front().theta)>ERRORRAD)
            {   
                // 枚举所有误差角度情况 对角度进行判断 进行速度限幅
               DeltaAngle=CurrentPoseEkf.theta-Static_Control.ExpectStack.front().theta;
                if((CurrentPoseEkf.theta>0)&& ((DeltaAngle>PI)&&(DeltaAngle<(2*PI))))          //PI 2PI
                     {
                                 new_twist.angular_z.data = Static_Control.Clientspeed.data;
                     }
                else if((CurrentPoseEkf.theta<0)&& ((DeltaAngle>(-2*PI))&&(DeltaAngle<(-1*PI)))) //-2PI -PI
                        {   
                                 new_twist.angular_z.data = -Static_Control.Clientspeed.data;
                        }
                else if((DeltaAngle>0)&&(DeltaAngle<PI))                                           //0 PI
                        {
                                 new_twist.angular_z.data = - Static_Control.Clientspeed.data;      
                        }
                else if((DeltaAngle<0)&&(DeltaAngle >(-PI)))                                        //-PI 0
                        {
                                 new_twist.angular_z.data = Static_Control.Clientspeed.data;
                        }
            }
            //现有odom - 静态odom 等于期望theta
            else if(ABS(CurrentPoseEkf.theta-Static_Control.ExpectStack.front().theta)<ERRORRAD)   
                {
                        memset(&new_twist,0,sizeof(new_twist));
                        if((Static_Control.ExpectStack.front().rotation==1)&&(Static_Control.ExpectStack.front().state==0))
                        {
                            Static_Control.ExpectStack.front().state=1;
                            auto iter = Static_Control.ExpectStack.erase(Static_Control.ExpectStack.begin());  
                        }else;
                }
        }
        else if((Static_Control.ExpectStack.front().rotation==0)&&(Static_Control.ExpectStack.front().state==0))
        {
            new_twist.angular_z.data= 0.0;
         //  ROS_INFO("----- delta distance is[%f],current length is[%f]",\
                     DistancePoint(TaskOld_pose,CurrentPoseEkf),\
                             Static_Control.ExpectStack.front().length);
            if(DistancePoint(TaskOld_pose,CurrentPoseEkf)<Static_Control.ExpectStack.front().length)
            {
                new_twist.liner_x.data=Static_Control.Clientspeed.data;  //二次降速
            }
            else 
            {
               memset(&new_twist,0,sizeof(new_twist));     
              if((Static_Control.ExpectStack.front().rotation!=true)&&(Static_Control.ExpectStack.front().state!=true))
                {
                        Static_Control.ExpectStack.front().state=true;
                        auto iter = Static_Control.ExpectStack.erase(Static_Control.ExpectStack.begin());
                }else;   
            }
        }
        else 
        { 
            // ROS_INFO("the fitst roatation is -----[%d],the state is [%d]",Static_Control.ExpectStack.front().rotation,Static_Control.ExpectStack.front().state);
                cout<<"------false "<<endl;
        }
          PublishCmdRtime(new_twist);
    }
    else                    //初始化停止
    {
        Initcount++;
       memset(&cureent_twist,0,sizeof(cureent_twist));
       if(Initcount==1)
       {
         PublishCmdRtime(cureent_twist);  
       }else ;
    }
    if(Static_Control.ExpectStack.size()==0)YdmsgsIndex->pose_state==true;else;
}
/** 
* @param   提取odom—pose 的odom位置信息 分化到 数组 pose2d 对数据预处理 作为反馈输入量
* @param  
* @param  
* @
eturn   pose 2d  current pose
*/
void  Robot_Controller::PoseOdom_Get(const nav_msgs::Odometry &odom_pose)
{
        tf::Matrix3x3 CurrentPoseOringin;                         
        tf::Quaternion quat_tf;                             //四元素
        tf::quaternionMsgToTF(odom_pose.pose.pose.orientation,quat_tf);  //geometry 四元素转化为tf
        double testyaw = tf::getYaw(quat_tf);               //提取yaw 
        vector<tf::Vector3> MartixPoseOdom;                 //定义一个三维向量队列 存储旋转矩阵
        CurrentPoseOringin.setRotation(quat_tf);           //四元素获取旋转旋转矩阵
        tfScalar OrinPoseYaw,OrinPosePitch,OrinPoseRoll;   //欧拉角
        MartixPoseOdom.push_back(CurrentPoseOringin[0]);       //存入旋转矩阵
        MartixPoseOdom.push_back(CurrentPoseOringin[1]);
        MartixPoseOdom.push_back(CurrentPoseOringin[2]);
        CurrentPoseOringin.getEulerYPR(OrinPoseYaw,OrinPosePitch,OrinPoseRoll);//旋转矩阵到欧拉角提取
        CurrentPoseOdom.x= odom_pose.pose.pose.position.x;
        CurrentPoseOdom.y= odom_pose .pose.pose.position.y;
        CurrentPoseOdom.theta = OrinPoseYaw;
 }
/** 
* @param   提取ekf—pose 的odom位置信息 分化到数组 pose2d 对数据预处理 作为反馈输入量
* @param   采样周期10ms 同时订阅
* @param  
* @             
eturn   pose 2d  current pose
*/
void Robot_Controller::PoseEkf_Get(const geometry_msgs::PoseWithCovarianceStamped &ekf_pose)				
{
        tf::Matrix3x3 EkfPoseMatrix;                      
        geometry_msgs::Quaternion EkfPoseReceive=ekf_pose.pose.pose.orientation;
        tf::Quaternion quat_tf;
        tf::quaternionMsgToTF(EkfPoseReceive,quat_tf);
        double testyaw = tf::getYaw(quat_tf);

        EkfPoseMatrix.setRotation(quat_tf); 
        tfScalar EkfPoseYaw,EkfPosePitch,EkfPoseRoll;
        vector<tf::Vector3> MartixPoseEkf;
        EkfPoseMatrix.getEulerYPR(EkfPoseYaw,EkfPosePitch,EkfPoseRoll);
        MartixPoseEkf.push_back(EkfPoseMatrix[0]);
        MartixPoseEkf.push_back(EkfPoseMatrix[1]);
        MartixPoseEkf.push_back(EkfPoseMatrix[2]);
        CurrentPoseEkf.x=ekf_pose.pose.pose.position.x; 
        CurrentPoseEkf.y=ekf_pose.pose.pose.position.y;
        CurrentPoseEkf.theta=EkfPoseYaw; 
}
/**
*  
* @param   // 设置位置 目标类型pose  返回实际坐标
* @param  
* @param  
eturn 
*/
 geometry_msgs::Pose2D Robot_Controller::Pose_Set(const geometry_msgs::Pose2D &SetPose)
{
    static  geometry_msgs::Pose2D Robot_Pose_Current;
    return Robot_Pose_Current;
}

float Robot_Controller::DistancePoint(geometry_msgs::Pose2D& a,geometry_msgs::Pose2D& b)
{
     return sqrt(pow((a.x-b.x),2)+pow((a.y-b.y),2));
}
float Robot_Controller::ThetaPoint(geometry_msgs::Pose2D& start,geometry_msgs::Pose2D& end)
{   
        return atan2((end.y-start.y),(end.x-start.x));     
}

void Robot_Controller::Speed_Set(const geometry_msgs::Twist &setwist)
{   
    Twistpublisher.publish(setwist);
}

void 	 Robot_Controller::TimetaksPub(const ros::TimerEvent &_event)
{
    
}
//initnode
void Robot_Controller::InitNodeHandle(void)
{
	// ros::NodeHandle private_nh
    // ("~");
    memset(&Static_Control,0,sizeof(Static_Control));  //
	// pose_publisher    = nh.advertise<geometry_msgs::Pose2D>("PoseState",100);  //发布pose state 状态  这个节点的主要功能
    Twistpublisher= nh.advertise<geometry_msgs::Twist>("cmd_vel",100);           //发布模糊控制速度 实际控制的  
    ControlSate    = nh.advertise<std_msgs::Bool>("ControState",100);
    BackToChargePublisher  =nh.advertise<std_msgs::Int8>("BackToC",10);
    
    Ekf_PoseSub = nh.subscribe("robot_pose_ekf/odom_combined",100, &Robot_Controller::PoseEkf_Get,this);//测量 输入量1
    Power_Sub = nh.subscribe("PowerVoltage",100,&Robot_Controller::BackToCharge,this); //订阅电量信息

    Odom_PoseSub = nh.subscribe("/odom",100,&Robot_Controller::PoseOdom_Get,this);    // 观察量 
	Pose_server  =  nh.advertiseService("Rm_TargetPose",&Robot_Controller::ServerTask,this);  //set a service name Rm_TargetPose
    ros::service::waitForService("/move_base/clear_costmaps");
    clear_client_base = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    clear_map.request={};
    TimeSpeed_ = nh.createTimer(ros::Duration(0.05), &Robot_Controller::TimetaksPub,this,false,true);  

    ROS_INFO("Rm_Server Open success "); 

 }

 void Robot_Controller::BackToCharge(const sensor_msgs::BatteryState &Now_Power_)
 {
    std_msgs::Int8 Power_Info; //不同id 确定不同工作状态
    if(Now_Power_.percentage<0.30 )
    {
            Power_Info.data=1;
            BackToChargePublisher.publish(Power_Info);    
    }
    else
    {
        Power_Info.data = 0;
        BackToChargePublisher.publish(Power_Info);
    }
     
 }
/*-------------------------------------
** @name:              
** @brief:             open for the other class obeject
** @param:             ctrl is the input pose  which we want  to set up
** @date:              2024-07-15
** @version:           V0.0
---------------------------------------*/
bool  Robot_Controller::Dynatic_Controller(const geometry_msgs::Pose2D &ctrl, float  cspeed)
{
    float orin_theta = 0.0;
    float orin_buffer = 0.0;
    TaskPoint LineTarget ;
    memset(&LineTarget,0,sizeof(LineTarget));
    if((ctrl.x==0.0)&&(ctrl.y==0.0)&&(ABS(ctrl.theta)!=0))
    {
            memset(&Start_Point,0,sizeof(Start_Point));
            Start_Point.theta = ctrl.theta;
            Start_Point.state=false;
            Start_Point.rotation= 1;
            Start_Point.length=0.0;
            Static_Control.ExpectStack.push_back(Start_Point);//存入任务点 包含任务目标和任务状态  
            return true;
    }else
    {
      if(Static_Control.ExpectStack.size()==0) 
        {
             memset(&ActionOldPose_,0,sizeof(ActionOldPose_));          
             ActionOldPose_=CurrentPoseEkf;                                  //现在位置复制
             target_pose.x=ActionOldPose_.x+ctrl.x;
             target_pose.y=ActionOldPose_.y+ctrl.y;
             target_pose.theta=ActionOldPose_.theta+ctrl.theta;
        }else
        {
            return false;
            //此处中断任务 或者进入优先级
        }
        orin_theta = atan2((ctrl.y),(ctrl.x));
        orin_buffer = orin_theta+ActionOldPose_.theta;
        orin_buffer= orin_buffer>PI? \
                       orin_buffer-(2*PI):\
                       orin_buffer<(-1 *PI)?\
                       orin_buffer+(2*PI):\
                       orin_buffer;
        Start_Point.theta = orin_buffer;
        Start_Point.length = 0.0;
        if(orin_buffer==0.0){Start_Point.rotation=0;}
        else{ Start_Point.rotation =1;}
        Start_Point.ID=0;
        Static_Control.distance=sqrt(pow((ctrl.x-ActionOldPose_.x),2)+pow((ctrl.y-ActionOldPose_.y),2));//path length 分割长度后续优化 长度判断符
        if(Static_Control.distance>3)  //3米 约束
        {
          //报错
        }else;
        Static_Control.ExpectStack.push_back(Start_Point);
        if(cspeed==0.0){Static_Control.Clientspeed.data=0.1;}else
        {Static_Control.Clientspeed.data= limit(-0.8,cspeed,0.8); }
        
        LineTarget = PathLine(ActionOldPose_,target_pose);
        Static_Control.ExpectStack.push_back(LineTarget);
       // End_Point.theta =  ctrl.theta - target_pose.theta;
        End_Point.theta = target_pose.theta;
        End_Point.theta = End_Point.theta >PI? \
                          End_Point.theta-(2*PI):\
                          End_Point.theta<(-1*PI)?\
                          End_Point.theta+(2*PI):\
                          End_Point.theta; 
        End_Point.rotation = 1;
        End_Point.length = 0.0;
        End_Point.state = false;
        Static_Control.ExpectStack.push_back(End_Point);

    }
    return true;
}
//
bool   Robot_Controller::Staus(void)
{	
    if(Static_Control.ExpectStack.size()!=0)
    {
        ROS_INFO("the task id is [%d]",Static_Control.ExpectStack.size());
    }
    if((Static_Control.ExpectStack.size()==0)&&\
        (TwistBase.linear.x==0)&&(TwistBase.angular.z==0))
        {
         return true;}
	else 
        {  return false;}
}

Robot_Controller::~Robot_Controller()
{
  ROS_INFO_STREAM("Shutting down");	
}

}