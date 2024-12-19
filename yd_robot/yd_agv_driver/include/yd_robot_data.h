/*
 * @Author: robot_luoer@163.com robot_luoer@163.com
 * @Date: 2024-04-02 10:37:58
 * @LastEditors: robot_luoer@163.com robot_luoer@163.com
 * @LastEditTime: 2024-04-02 10:40:32
 * @FilePath: 
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __HG_ROBOT_DATA_H_
#define __HG_ROBOT_DATA_H_

#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include "yd_robot.h"
using namespace std;
#define  SeparatePower  (1.63/21.68)   //分压电路 硬件电压跟随器如果出错那么修改参数 优化电路 使用别家电池后放弃
int Auto_ston(const char *str);
char *Find_String(char *txt, char *ch,char *Repro_data);

float InvSqrt(float number);
void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az);

float AdcDataTrans(short data);
// char *Find_Stringl(char *txt, char *chl, char *chr, char *repro_data);
//void Float_Tchar(float *input, unsigned char *output);


// ________________________________ 速度结算        
/*
正解算
 vl vr :[  ]

*/
// ________________________________


#endif