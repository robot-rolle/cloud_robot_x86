
#include "yd_robot.h"
#include "yd_robot_data.h"
sensor_msgs::Imu  Gy95t;
int main(int argc, char** argv)
{
	ros::init(argc,argv,"yd_robot"); //发布机器人话题
	launch_agv  yd_agv;//创建机器人对象
	yd_agv.Task();//任务循环
	return 0;
}


void launch_agv::Task()
{			
	static float buffer_left=0,buffer_right=0;
	static float count_info=0;

	_Last_Time = ros::Time::now();  //定义机器人时间
	while (ros::ok())  //机器人系统存在继续循环
	{
	 _Now = ros::Time::now();	  
                                                 //获取时间间隔，用于积分速度获得位移(里程)s			 
	 while(true==Esp32_Serial.waitReadable())  //串口有数据
	 {												 
		if(2==Get_Sensor_Esp())  //串口是底板数据
		{ 

			Robot_Vel.t = (_Now -_Last_Time).toSec(); //Retrieves time interval, which is used to integrate velocity to obtain displacement (mileage) 

			Robot_Pos.X+=Robot_Vel.vc * cos(Robot_Vel.Angle) * Robot_Vel.t; //Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m 速度分解后 积分速度得到位移
			Robot_Pos.Y+=Robot_Vel.vc * sin(Robot_Vel.Angle) * Robot_Vel.t; //Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m速度分解后 积分速度得到位移
			Robot_Vel.Angle += Robot_Vel.w * Robot_Vel.t; //The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad   Robot_Vel是角速度，即 yaw轴的速ß度
			Robot_Pos.Z = Robot_Vel.Angle;

			count_info++;
		    Quaternion_Solution(Gy95t.angular_velocity.x,Gy95t.angular_velocity.y,Gy95t.angular_velocity.z,\
			Gy95t.linear_acceleration.x, Gy95t.linear_acceleration.y, Gy95t.linear_acceleration.z);  //发布陀螺仪进行转换
			Publish_Imu();  //发布imu话题
			Publish_Odom(); //发布里程计
			Publish_Voltage();  //发布电压

			_Last_Time = _Now; //Record the time and use it to calculate the time interval //记录时间，用于计算时间间隔       
			if(count_info>10)
			{
				count_info=0; 
			}
						//ROS_INFO("O:w:[%f],angle:[%f],x:[%f],y:[%f],t:[%f],speedl:[%f],speedr:[%f]",Robot_Vel.w,Robot_Pos.Z,Robot_Pos.X,Robot_Pos.Y,Robot_Vel.t,Robot_Vel.vl,Robot_Vel.vr);	
						// ROS_INFO("angle:[%f]",Robot_Pos.Z);	 

			break;
		}
		
	 }

	#if UWB_STATE
	ROS_INFO("--------------------read uwb----------");	
			while(true==Uwb_Serial.waitReadable())  //uwb数据是真
			{
				if(true==Get_Sensor_Uwb())//获取uwb数据
				{
					Publish_Uwb(); //发布uwb数据
					break;
				}											//此处添加超时退出
			}
	#endif				
      ros::spinOnce();   //The loop waits for the callback function //循环等待回调函数  
	}
}

/*
 * Functionname: launch_agv::Position_Check()
 * Author: Rolle
 * Email: 
 * Function: Position data 
 * 功能 ：位置信息融合
*/
void  launch_agv::Position_Check()
{
	
}
/*
 * Functionname: launch_agv::Check_Sum_CRC16
 * Author: Rolle
 * Email: 
 * Function:input the data, the length of the data, mode of crc
 * 功能 ：数据检验函数， 此处使用CRC16进行校验 输入数据地址， 数据长度，模式：0 转换crc, 模式：1 检验crc
*/
unsigned short launch_agv::Check_Sum_CRC16(unsigned char *Data, unsigned int  length,int mode) 
{
	unsigned short Crc_Data = 0xFFFF; // 
	unsigned short Crc_Check = 0xa001; // 0x8005
	//nsigned short Crc_Buff  = 0x0000;
	int i ,j ;
	if(mode == 0) //转换crc
	{
		if(Data !=NULL) //检验非空
		{
			for(i = 0; i<length; i++)  //循环输入
			{
				Crc_Data ^= Data[i];
				for(j = 0; j<8 ; j++)
				{
					if(Crc_Data & 0x0001)
					{
						Crc_Data = (Crc_Data >>1)^Crc_Check;
					}
					else
						Crc_Data >>=1; //ss
				}
			}//
		}	
	} 
	if(mode == 1)
	{
		if(Data != NULL)
		{
			for(i =0; i<length; i++)
			{
				Crc_Data ^=Data[i];
				for(j = 0 ; j < 8; j++)
				{
					if(Crc_Data & 0x0001)
					{
						Crc_Data = (Crc_Data >>1)^ Crc_Check;
					}
					else
						Crc_Data >>=1;
				}
			}	
		}
		else 
		;
	}
	return Crc_Data;
}

/*
 * Functionname: launch_agv::Publish_Uwb()
 * Author: Rolle
 * Email: 
 * Function: Publish Uwb
 * 功能 ：发布UWB 话题
*/
void  launch_agv::Publish_Uwb()
{
  std_msgs::Float32MultiArray Uwb_Pos;
  Uwb_Pos.data.resize(2);
  Uwb_Pos.data[0] = Receive_Data_Uwb.X_Position;
  Uwb_Pos.data[1] = Receive_Data_Uwb.Y_Position;
  uwb_publisher.publish(Uwb_Pos);
}
/*
 * Functionname: launch_agv::Publish_Voltage()
 * Author: Rolle
 * Email: 
 * Function: Publish Voltage
 * 功能 ： 发布电压 话题
*/
void  launch_agv::Publish_Voltage() 
{
	    _Power.header.stamp = ros::Time::now(); 
		_Power.voltage=AdcDataTrans(Power_voltage);
		_Power.header.frame_id ="car_voltage";
		_Power.percentage= ((AdcDataTrans(Power_voltage) -20.0)/ 9.4);//29.4 -20 /9.4 = 100%
		static float Count_Voltage_Pub=0;
		if(Count_Voltage_Pub++>=8)
		{
			Count_Voltage_Pub = 0;
			voltage_publisher.publish(_Power);
		}
}
/*
 * Functionname: launch_agv::Publish_Odom()
 * Author: Rolle
 * Email: 
 * Function: input the data of the odom
 * 功能 ：坐标系数据输入
*/
void  launch_agv::Publish_Odom()
{
  //Convert the Z-axis rotation Angle into a quaternion for expression
    //把Z轴转角转换为四元数进行表达
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);

    nav_msgs::Odometry odom; //Instance the odometer topic data //实例化里程计话题数据
    odom.header.stamp = ros::Time::now(); 
    odom.header.frame_id = odom_frame_id; // Odometer TF parent coordinates //里程计TF父坐标
    odom.pose.pose.position.x = Robot_Pos.X; //Position //位置
    odom.pose.pose.position.y = Robot_Pos.Y;
	//------test odom value viusal-----------
	// odom.pose.pose.position.x= Robot_Vel.vl;
	// odom.pose.pose.position.y =Robot_Vel.vr;

    odom.pose.pose.position.z = 0.085;//Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat; //Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

    odom.child_frame_id = robot_frame_id; // Odometer TF subcoordinates //里程计TF子坐标
    odom.twist.twist.linear.x =  Robot_Vel.vc; //Speed in the X direction //X方向速度
    odom.twist.twist.linear.y =  0; //Speed in the Y direction //Y方向速度
    odom.twist.twist.angular.z = Robot_Vel.w; //Angular velocity around the Z axis //绕Z轴角速度 

    //There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
	//ROS_INFO("twist x [%f] y [%f]",Robot_Vel.vc,Robot_Vel.w);
    if(Robot_Vel.vc== 0&&Robot_Vel.Y== 0&&Robot_Vel.w== 0)
      //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
      //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
    {  memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
      memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));}
    else
      //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
      //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
 	{ 	 memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
   		 memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance)); } 
						 
    odom_publisher.publish(odom); //Pub odometer topic //发布里程计话题
}
/*
 * Functionname: 
 * Author: Rolle
 * Email: 
 * Function: 
 * 功能 ：
*/
void  launch_agv::Publish_Imu()
{
	sensor_msgs::Imu  Imu_Data_Pub;
	float yaw;
	Imu_Data_Pub.header.stamp = ros::Time::now();
	Imu_Data_Pub.header.frame_id= gyro_frame_id;

	Imu_Data_Pub.orientation_covariance[0]=-1;  // 
 	Imu_Data_Pub.orientation.x = Gy95t.orientation.x; //A quaternion represents a three-axis attitude //四元数表达三轴姿态
 	Imu_Data_Pub.orientation.y = Gy95t.orientation.y; 
 	Imu_Data_Pub.orientation.z = Gy95t.orientation.z;
 	Imu_Data_Pub.orientation.w = Gy95t.orientation.w;
 	Imu_Data_Pub.orientation_covariance[0] = 1e6; //Three-axis attitude covariance matrix //三轴姿态协方差矩阵
 	Imu_Data_Pub.orientation_covariance[4] = 1e6;
 	Imu_Data_Pub.orientation_covariance[8] = 1e-6;
 	Imu_Data_Pub.angular_velocity.x = Gy95t.angular_velocity.x; //Triaxial angular velocity //三轴角速度
 	Imu_Data_Pub.angular_velocity.y = Gy95t.angular_velocity.y;
 	Imu_Data_Pub.angular_velocity.z = Gy95t.angular_velocity.z;
 	Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; //Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
 	Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
 	Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
 	Imu_Data_Pub.linear_acceleration.x = Gy95t.linear_acceleration.x; //Triaxial acceleration //三轴线性加速度
 	Imu_Data_Pub.linear_acceleration.y = Gy95t.linear_acceleration.y; 
 	Imu_Data_Pub.linear_acceleration.z = Gy95t.linear_acceleration.z;
	// Imu_Data_Pub.linear_acceleration_covariance[0]=-1;
  	//imu_publisher.publish(Imu_Data_Pub); //Pub IMU topic //发布IMU话题

	yaw = atan2(2 * (Gy95t.orientation.x * Gy95t.orientation.y + Gy95t.orientation.w * Gy95t.orientation.z), 
            Gy95t.orientation.w * Gy95t.orientation.w + Gy95t.orientation.x * Gy95t.orientation.x - Gy95t.orientation.y * Gy95t.orientation.y - Gy95t.orientation.z * Gy95t.orientation.z);
	//ROS_INFO("%f",yaw);			
}

/**
 * @Input : 
 * 机器人移动	moveXY:	float,	float,	LEN,\n	
	X旋转速度 圈/s	移动速度Y M/s ,Return:
 * @Functonname :
 * @MethodAuthor: Rolle,
 * @Description : 
 * @功能 :
*/
void launch_agv::Cmd_Vel_Callback(const geometry_msgs::Twist  &twist_aux)
{
	 short  buffer_data;  //intermediate variable //中间变量
	 float  datal,datar,vl,vr;
	 unsigned short crcdata;
	 uint8_t CRCA,CRCB;
	 uint8_t *in_buffer0=(uint8_t *)(&Send_Data_ST); //定义一个指向 32发送到ros端的结构体 指针
	 //运动学逆解算。vc 和 角速度 解算两个车轮速度
	 vl =(twist_aux.linear.x - (WHEELWIDTH /2) * twist_aux.angular.z ); //
	 vr =(twist_aux.linear.x + (WHEELWIDTH /2) * twist_aux.angular.z );
	//线速度转换为圈数下发
	datal = vl / (PI * WHEELLENGTH);
	datar = vr / (PI * WHEELLENGTH);
	//限制圈速  下发rps的时候进行转换
	// datar = -(limit(-3.0,datar,3.00)); 
	// datal = limit(-3.0,datal,3.00);
	//yd 机器人使用 线速度限制幅度
	
	datal = limit(-3,datal,3.0);
	datar = -(limit(-3,datar,3.0)); 
	buffer_data=0;
	Send_Data_ST.txesp[0]=FRAME_HEADER_A;
	Send_Data_ST.txesp[1]=FRAME_HEADER_B;	
	Send_Data_ST.txesp[2]=0x01;//数据包1
	buffer_data=datal*1000;
	Send_Data_ST.txesp[3]=buffer_data>>8;	  //left speed 
	Send_Data_ST.txesp[4]=buffer_data&0xff;
	buffer_data=0;
	buffer_data=datar*1000;	
	Send_Data_ST.txesp[5]=buffer_data>>8;	
	Send_Data_ST.txesp[6]=buffer_data&0xff;
	Send_Data_ST.txesp[7]=0x01;	  //数据有效
	crcdata=Check_Sum_CRC16(in_buffer0+3,SEND_DATA_SIZE-7,0);//crc提取
	CRCA = crcdata>>8;
	CRCB = crcdata&0x00ff;	
	Send_Data_ST.txesp[8]=CRCB;	
	Send_Data_ST.txesp[9]=CRCA;	

	Send_Data_ST.txesp[10]=FRAME_TAIL_A;
	Send_Data_ST.txesp[11]=FRAME_TAIL_B;


	//发布字符串
	#if SEND_METHOD
	string  data_send ="SpeedLR:"+std::to_string(datal)+","+std::to_string(datar)+",\n";
	const  char *hg_por =data_send.c_str();
	#endif

	 try
	 {
	  Esp32_Serial.write(Send_Data_ST.txesp,sizeof(Send_Data_ST.txesp)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
      //ROS_INFO("speed l"); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
		//cout<<"______left:   "<<datal<<"____right:  "<<datar<<endl;
	 }
	 catch (serial::IOException& e)   
	 {
	   ROS_ERROR_STREAM("Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
	 }
}
/**
 * @Input :  ,Return:
 * @Functonname :
 * @MethodAuthor: Rolle,
 * @Description : 
 * @功能 :
*/
int launch_agv::Get_Sensor_Esp()
{
    int length=0;                         //定义接受数组下标 define the receive array index
    // short transition_16 =0;                 // 转换字符
    unsigned short crcdata;
    uint8_t Receive_Data_Serial[1],CRCA,CRCB;   //定义字符串缓冲器 
    static int count = 0;             //静态计数器 state =0  state =1
	float Roll,Yaw,Pitch;
	
    Esp32_Serial.read(Receive_Data_Serial,sizeof(Receive_Data_Serial));  //读取串口数据 一个字节存入缓冲器 
    Receive_Data_Esp.rxesp[count]=Receive_Data_Serial[0];     //数据存入 缓存数组中

	if(Receive_Data_Serial[0]==FRAME_HEADER_A || Receive_Data_Serial[0]==FRAME_HEADER_B || count >1) //判断是头帧还是 中间数据 是的话 存入 不是 清零 重新读
	 {count++;
	 }
	else
	 count =0;
	//  ROS_INFO("Receive_Data_Serial is : %d",Receive_Data_Serial[0]);
	if((Receive_Data_Serial[0]==FRAME_TAIL_B)&&(Receive_Data_Esp.rxesp[count-2]==FRAME_TAIL_A))//判断是否是尾帧
	{
	
		if((count==RECEIVE_DATA_LENGTH_A)&&(Receive_Data_Esp.rxesp[2]==0x00))  //判断数据包是 data0 还是 data1 而且数据包长度正确
		{
			for (int i = 3; i <count-4; i++)							//存入自定义数组 data0
			{
				 Receive_Data_Esp.rxdata0[i-3]=Receive_Data_Esp.rxesp[i];
			}
			length=RECEIVE_DATA_LENGTH_A-7;
			crcdata=launch_agv::Check_Sum_CRC16(Receive_Data_Esp.rxdata0,length,1); //crc 检验码获取
			CRCA = crcdata>>8;
			CRCB = crcdata&0x00ff;																//分离crc 前后
			if((CRCA==Receive_Data_Esp.rxesp[count-3])&& (CRCB==Receive_Data_Esp.rxesp[count-4])) //crc正确 存入
			{
				count =0;										//静态计数器清零
				for(int i =0; i<=length;i++)									//
				{
					databuffer0[i] = Receive_Data_Esp.rxdata0[i];
				}
				return 1;
 			}
		}
		else if((count==RECEIVE_DATA_LENGTH_B)&&(Receive_Data_Esp.rxesp[2]==0x01))//判断数据包是 data0 还是 data1
		{	
			for (int i = 3; i <count-4; i++)				//存入自定义数组 data1
			{
				 Receive_Data_Esp.rxdata1[i-3]=Receive_Data_Esp.rxesp[i]; 
				// cout<<"data is :%d",<<Receive_Data_Esp.rxesp[i]<<endl;
				//  ROS_INFO("-----------: %d",Receive_Data_Esp.rxesp[i]);
			}
			length=RECEIVE_DATA_LENGTH_B-7;
			crcdata=launch_agv::Check_Sum_CRC16(Receive_Data_Esp.rxdata1,length,1); //crc 
			CRCA = crcdata>>8;
			CRCB = crcdata&0x00ff;
			if((CRCA==Receive_Data_Esp.rxesp[count-3])&& (CRCB==Receive_Data_Esp.rxesp[count-4])) //crc正确 存入数据
			{
				count =0;
				for(int i=0;i<=length;i++)				
				{
					databuffer1[i]=Receive_Data_Esp.rxdata1[i];   //地址复制 数据
					//ROS_INFO("%d:",databuffer1[i]);				
				}	
				// ROS_INFO("-----------:CRC success------------");
			 	// if((Esp_Origin_Data.data1.motorSpeeds[0]>-1)&&(Esp_Origin_Data.data1.motorSpeeds[0]<2)) {Esp_Origin_Data.data1.motorSpeeds[0]=0;}
				// else;
				// if ((Esp_Origin_Data.data1.motorSpeeds[1]>-1)&&(Esp_Origin_Data.data1.motorSpeeds[1]<2)) {Esp_Origin_Data.data1.motorSpeeds[1]=0;}
				// else;
				

				Robot_Vel.vl = -Esp_Origin_Data.data1.motorSpeeds[0] * PI * WHEELLENGTH ;//左轮速度提取， 圈速 转换成线速度
				Robot_Vel.vr = Esp_Origin_Data.data1.motorSpeeds[1] * PI * WHEELLENGTH ;//右轮速度提取，圈速转换成线速			

				Robot_Vel.vc = (Robot_Vel.vr + Robot_Vel.vl)/ 2;  //速度解算 得到中心 坐标线速度	
			//	Robot_Vel.vr=(-0.01,Robot_Vel.vr,0.01);
				// cout<<" l  speed ： "<<Robot_Vel.vl <<"    r speed：    "<<Robot_Vel.vr <<endl;
				// ROS_INFO("----------x:%d , y:%d ,z:%d,f:%d",Receive_Data_Esp.rxdata1[16],Receive_Data_Esp.rxdata1[17],Receive_Data_Esp.rxdata1[18],Receive_Data_Esp.rxdata1[19]);
				//ROS_INFO("----------x:%f , y:%f",Robot_Vel.vl,Robot_Vel.vr);	


				Robot_Vel.w =(Robot_Vel.vr-Robot_Vel.vl) / (WHEELWIDTH);  // 速度解算 得到角速度
			
				if((Esp_Origin_Data.data1.motorSpeeds[0]==Esp_Origin_Data.data1.motorSpeeds[1])&&(Esp_Origin_Data.data1.motorSpeeds[1]==0))
				{
					Robot_Vel.w = 0;		//当左右速度为0 角速度设置为0
				}
				Gy95t.linear_acceleration.x = Esp_Origin_Data.data1.acc[0]/ACCEl_PARAM;		 // 提取线加速度 除以精度
			    Gy95t.linear_acceleration.y = Esp_Origin_Data.data1.acc[1]/ACCEl_PARAM;		
				Gy95t.linear_acceleration.z = Esp_Origin_Data.data1.acc[2]/ACCEl_PARAM;    
				Gy95t.angular_velocity.x =  Esp_Origin_Data.data1.gyro[0] * GYROSCOPE_PARAM; //提取角速度x	乘上精度
				Gy95t.angular_velocity.y =  Esp_Origin_Data.data1.gyro[1] * GYROSCOPE_PARAM; //提取角速度y  乘上精度
				Gy95t.angular_velocity.z =  Esp_Origin_Data.data1.gyro[2] * GYROSCOPE_PARAM; //提取角速度z  乘上精度
				Power_voltage = Esp_Origin_Data.data1.Power;
				// ROS_INFO("----------x:%d ",Power_voltage);		
				return 2;
			}
			else 
				return -21;
		}
		else if((count==RECEIVE_DATA_LENGTH_C)&&(Receive_Data_Esp.rxesp[2]==0x02))//判断数据包是 data0 还是 data1 data2
		{	
			for (int i = 3; i <count-4; i++)				//存入自定义数组 data2
			{
				 Receive_Data_Esp.rxdata2[i-3]=Receive_Data_Esp.rxesp[i];
			}
			length=RECEIVE_DATA_LENGTH_C-7;
			crcdata=launch_agv::Check_Sum_CRC16(Receive_Data_Esp.rxdata2,length,1); //crc 
			CRCA = crcdata>>8;
			CRCB = crcdata&0x00ff;
			if((CRCA==Receive_Data_Esp.rxesp[count-3])&& (CRCB==Receive_Data_Esp.rxesp[count-4])) //crc正确 存入数据
			{
				count =0;
				for(int i=0;i<=length;i++)				
				{
					databuffer2[i]=Receive_Data_Esp.rxdata2[i];
				}	
				// 超声波数据处理 
				return 3;
			}
		}
		else
		{
				count =0;
			 	return -12;	
		}
	}
	return -1;
}

/*
 * Functionname: Get_Sensor_Uwb
 * Author: Rolle
 * Email: 
 * 
 * Function: 
*/
bool launch_agv::Get_Sensor_Uwb()
{
    int buffer = 0;                         //定义接受数组下标 define the receive array index
    // short transition_16 =0;                 // 转换字符
    unsigned char Receive_Data_Uwb_Pr[1];   //定义字符串缓冲器
    static int count =0,state =0;             //静态计数器
	char *pro_index;
	char puck[15];
    Uwb_Serial.read(Receive_Data_Uwb_Pr,sizeof(Receive_Data_Uwb_Pr));  //读取串口数据 一个字符
	if(Receive_Data_Uwb_Pr[0]!='\n')
	{
   		Receive_Data_Uwb.rxuwb[count] = Receive_Data_Uwb_Pr[0];                     //存入读取的字符到结构体
		state = 0;
	}
	else
	{
		state =1;													//设定状态清0
		Receive_Data_Uwb.rxuwb[count]='\n';								//尾镇
		pro_index = Receive_Data_Uwb.rxuwb;					//
		pro_index = Find_String(pro_index,(char *)"=",puck);
		if(strcmp(puck,"Rtls:X ")==0)
		{
			pro_index =Find_String(pro_index+2,(char *)" ",puck);
			Receive_Data_Uwb.X_Position= atof(puck)*0.01;
			pro_index =Find_String(pro_index+1,(char *)"=",puck);
			if(strcmp(puck,"cm , Y ")==0)
			{
				pro_index=Find_String(pro_index+2,(char *)" ",puck);
				Receive_Data_Uwb.Y_Position= atof(puck)*0.01;	
				pro_index=Find_String(pro_index+1,(char *)"m",puck);
				if(strcmp(puck,"c")==0)
				{	
					count = 0;
					memset(&Receive_Data_Uwb.rxuwb,0,sizeof(Receive_Data_Uwb.rxuwb));
					memset(&pro_index,0,sizeof(pro_index));					
					Receive_Data_Uwb.length=count;
					return true;
				}
				else {count =0;}	
			} else count=0;
		}
		else 
		{
			count=0;
		}
	}
	if((((Receive_Data_Uwb.rxuwb[0]=='R')&&(Receive_Data_Uwb.rxuwb[1]=='t'))||((Receive_Data_Uwb_Pr[0]=='R')&& (count == 0)))&&(state==0))//协议判断
     { 
		 count++;
	 }
	else
       count = 0;  
      return false;
}

void  launch_agv::Reset_Send(SEND_DATA *stm_cmd,uint8_t state,float vl,float vr)
{
	//*(stm_cmd->txesp+1)=FRAME_HEADER_A;//	(*stm_cmd).txesp[0]=FRAME_HEADER_A;
    *(stm_cmd->txesp)=FRAME_HEADER_A;
}

/**
 * @Input : event ,Return:void
 * @Functonname :
 * @MethodAuthor: Rolle,
 * @Description : 
 * @功能 :定时器回掉函数 ____________________测试使用
*/
void launch_agv::time_test(const ros::TimerEvent & event)
{

}
/*
 * Date: 2022-07-19 12:18:15
 * Functionname:launch_agv::launch_agv()
 * Author: Rolle
 * Email: sdsds
 * Function: 
 * 功能 ：析构函数，实体化对象调用一次。
*/
launch_agv::launch_agv():Sampling_Time(0)
{
	//init the data of all
	memset(&Robot_Pos,0,sizeof(Robot_Pos));
	memset(&Robot_Vel,0,sizeof(Robot_Vel));
	memset(&Send_Data_ST,0,sizeof(Send_Data_ST));
	memset(&Receive_Data_Esp,0,sizeof(Receive_Data_Esp));
	memset(&Receive_Data_Uwb,0,sizeof(Receive_Data_Uwb));
	memset(&Esp_Origin_Data,0,sizeof(Esp_Origin_Data));
	memset(&Gy95t,0,sizeof(Gy95t));
	// string  Init_odom ="AGVDrive_Init:,\n";  //初始化  32 进行判断 否则地盘不运行
	// const  char *Init_odomc =Init_odom.c_str();
	ROS_INFO("--------------------start-----------");	
	timeout_uwb=5; //uwb 检测1ms 没有信息 就不发布 uwb
	timeout_esp=10; // esp 检测2 ms 没有信息就不发布esp 对应的话题
	//set the topic name and topic  these link the launch file创建话题节点。同时定义命名usb文件
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("usart_port_name",usart_port_name,"/dev/yd_agv_control"); //串口信息，自定义串口规则和 串口id
	private_nh.param<std::string>("odom_frame_id",odom_frame_id,"odom_combined"); //机器人里程计 坐标系
	private_nh.param<std::string>("robot_frame_id",robot_frame_id,"base_footprint"); //机器人中心坐标系
	private_nh.param<std::string>("usart_port_name_uwb",usart_port_name_uwb,"/dev/ydrobot_uwb"); // uwb串口名
	private_nh.param<std::string>("gyro_frame_id",gyro_frame_id,"gyro_link");  //节点坐标

	private_nh.param<int>("serial_baud_rate_uwb",serial_baud_rate_uwb,115200); //串口 波特率 设定， 外部launch 使用直接调用
	private_nh.param<int>("serial_baud_rate",serial_baud_rate,115200); //串口波特率
	// publisher  size
	voltage_publisher = n.advertise<sensor_msgs::BatteryState>("PowerVoltage",100); //发布话题 电压

	odom_publisher    = n.advertise<nav_msgs::Odometry>("odom",50);  //坐标系 信息话题

	// imu_publisher     = n.advertise<sensor_msgs::Imu>("imu",20);  //imu坐标话题 发布设置
	uwb_publisher	  = n.advertise<std_msgs::Float32MultiArray>("Uwb2",200);  //uwb 话题建立
	// pose_publisher    = n.advertise<geometry_msgs::Pose2D>("PoseState",100); //pose话题发布 绝对位置
	// ros::Publisher pub = n.advertise<nav_msgs::Odometry>("carto_odom_test", 1);//new test
	
	timer1 = n.createTimer(ros::Duration(0.5), &launch_agv::time_test,this,false,true);  //设定一个定时器， 20ms 调用一次 回掉函数time_test  是否使用一次false， 循环调用true

	Cmd_Vel_Sub 	  = n.subscribe("cmd_vel",100,&launch_agv::Cmd_Vel_Callback,this); //速度话题订阅  订阅共用cmd_vel

	
	ROS_INFO_STREAM("Launch_agv  is ready operate");  //ros打印
	try
	{
		Esp32_Serial.setPort(usart_port_name);   //设定ros串口名字
		Esp32_Serial.setBaudrate(serial_baud_rate); //设定esp32 波特率
		serial::Timeout _time =  serial::Timeout::simpleTimeout(2000);//2s设定 
		Esp32_Serial.setTimeout(_time);									//设定2s		
		Esp32_Serial.open();			// 打开串口

		serial::Timeout _out_time_esp =  serial::Timeout::simpleTimeout(timeout_esp);  //开启后 多久没检测到数据 报错
	  	Esp32_Serial.setTimeout(_out_time_esp);  //设定超时时间  这个影响 ros 数据读取效率
	#if  UWB_STATE  //测试屏蔽uwb
		Uwb_Serial.setPort(usart_port_name_uwb);       
		Uwb_Serial.setBaudrate(serial_baud_rate_uwb);
		serial::Timeout _time_=serial::Timeout::simpleTimeout(2000);
		Uwb_Serial.setTimeout(_time_);
		Uwb_Serial.open();

		serial::Timeout _out_time_uwb=serial::Timeout::simpleTimeout(timeout_uwb); //开启后 多久没检测到数据 报错
		Uwb_Serial.setTimeout(_out_time_uwb);
	#endif
	}
	catch (serial::IOException& e)
	{
		if (!Esp32_Serial.isOpen())
		{
			ROS_ERROR_STREAM("can not oepn Esp32_Serial port,please check the port cable or the rules");
		}
	#if  UWB_STATE  //测试屏蔽uwb
		else if (!Uwb_Serial.isOpen())
		{
			ROS_ERROR_STREAM("can not open Uwb_Serial port,please check the port cable or the rules ");
		}
	#endif
		else
		{
			ROS_ERROR_STREAM("all port error,please check serial"); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
		}	
	}
	if(Esp32_Serial.isOpen())
	{
		ROS_INFO_STREAM("Esp32_Serial  port opened successful"); //Serial port opened successfully //串口开启成功提示
	}
	#if  UWB_STATE  //测试屏蔽uwb
	if(Uwb_Serial.isOpen())
	{
		ROS_INFO_STREAM("Uwb_Serial serial port of uwb opened :successful");
	}	
	#endif
	//  try
	//  {
	//   Esp32_Serial.write(Init_odomc); //Sends data to the downloader via serial port //通过串口向下位机发送数据    初始化防止上电疯转 32接受到后才开始允许发送电机指令
	//  }
	//  catch (serial::IOException& e)   
	//  {
	//    ROS_ERROR_STREAM("Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
	//  }
}

/*
 * Functionname: launch_agv::~launch_agv()
 * Author: Rolle
 * Email: 
 * Function: 
 * 功能 ：
*/
launch_agv::~launch_agv()
{

	// string hg ="SpeedLR:"+std::to_string(0.000)+","+std::to_string(0.000)+",\n";//结束符号
	// const  char *fuck = hg.c_str();//同上，要加const或者等号右边用char*		
	short  buffer_data;  //intermediate variable //中间变量
	 unsigned short crcdata;
	 uint8_t CRCA,CRCB;
	 uint8_t *in_buffer0=(uint8_t *)(&Send_Data_ST); //定义一个指向 32发送到ros端的结构体 指针
	buffer_data=0;
	memset(Send_Data_ST.txesp,0,sizeof(Send_Data_ST.txesp));
	Send_Data_ST.txesp[0]=FRAME_HEADER_A;
	Send_Data_ST.txesp[1]=FRAME_HEADER_B;	
	Send_Data_ST.txesp[2]=0X01;
	Send_Data_ST.txesp[7]=0x00; //32不要串口发数据了
	crcdata=Check_Sum_CRC16(in_buffer0+3,SEND_DATA_SIZE-7,0);//crc提取
	CRCA = crcdata>>8;
	CRCB = crcdata&0x00ff;	
	Send_Data_ST.txesp[8]=CRCB;	
	Send_Data_ST.txesp[9]=CRCA;	
	Send_Data_ST.txesp[10]=FRAME_TAIL_A;
	Send_Data_ST.txesp[11]=FRAME_TAIL_B;
	try
	{
	   Esp32_Serial.write(Send_Data_ST.txesp,sizeof(Send_Data_ST.txesp)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
	   Esp32_Serial.write(Send_Data_ST.txesp,sizeof(Send_Data_ST.txesp)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 

	}
	catch(serial::IOException& e)   
	{
  		  ROS_ERROR_STREAM("Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
	}
	Esp32_Serial.close();
	#if  UWB_STATE  //测试屏蔽uwb
	Uwb_Serial.close();
	#endif
	ROS_INFO_STREAM("-------Shutting down----------");	
}
