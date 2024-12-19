/*
 * @Author: robot_luoer@163.com robot_luoer@163.com
 * @Date: 2022-08-02 10:40:21
 * @LastEditors: robot_luoer@163.com robot_luoer@163.com
 * @LastEditTime: 2022-08-02 10:43:52
 * @FilePath: /hg_robot/hg_first/src/hg_robot_agv/src/hg_robot_data.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "yd_robot_data.h"
#include "yd_robot.h"

//#define SAMPLING_FREQ 50.0f // 采样频率
#define SAMPLING_FREQ 18.0f // 采样频率
/**************************************
Function: 平方根倒数 求四元数用到
***************************************/

float InvSqrt(float number)
{
  volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );

  return y;
}
/**************************************
Date: May 31, 2020
Function: 四元数解算
***************************************/
// volatile float twoKp = 1.0f;     // 2 * proportional gain (Kp)
// volatile float twoKi = 0.0f;     // 2 * integral gain (Ki)
// volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
// volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
// void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az)
// {
//   float recipNorm;
//   float halfvx, halfvy, halfvz;
//   float halfex, halfey, halfez;
//   float qa, qb, qc;
//   float yaw;
//   // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//   if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
//     // 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
//     recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
//     ax *= recipNorm;
//     ay *= recipNorm;
//     az *= recipNorm;      
//     // 把四元数换算成方向余弦中的第三行的三个元素
//     halfvx = q1 * q3 - q0 * q2;
//     halfvy = q0 * q1 + q2 * q3;
//     halfvz = q0 * q0 - 0.5f + q3 * q3;
//     //误差是估计的重力方向和测量的重力方向的交叉乘积之和
//     halfex = (ay * halfvz - az * halfvy);
//     halfey = (az * halfvx - ax * halfvz);
//     halfez = (ax * halfvy - ay * halfvx);
//     // 计算并应用积分反馈（如果启用）
//     if(twoKi > 0.0f) {
//       integralFBx += twoKi * halfex * (1.0f / SAMPLING_FREQ);  // integral error scaled by Ki
//       integralFBy += twoKi * halfey * (1.0f / SAMPLING_FREQ);
//       integralFBz += twoKi * halfez * (1.0f / SAMPLING_FREQ);
//       gx += integralFBx;        // apply integral feedback
//       gy += integralFBy;
//       gz += integralFBz;
//     }
//     else {
//       integralFBx = 0.0f;       // prevent integral windup
//       integralFBy = 0.0f;
//       integralFBz = 0.0f;
//     }
//     // Apply proportional feedback
//     gx += twoKp * halfex;
//     gy += twoKp * halfey;
//     gz += twoKp * halfez;
//   }
//   // Integrate rate of change of quaternion
//   gx *= (0.5f * (1.0f / SAMPLING_FREQ));   // pre-multiply common factors
//   gy *= (0.5f * (1.0f / SAMPLING_FREQ));
//   gz *= (0.5f * (1.0f / SAMPLING_FREQ));
//   qa = q0;
//   qb = q1;
//   qc = q2;
//   q0 += (-qb * gx - qc * gy - q3 * gz);
//   q1 += (qa * gx + qc * gz - q3 * gy);
//   q2 += (qa * gy - qb * gz + q3 * gx);
//   q3 += (qa * gz + qb * gy - qc * gx); 
//   // Normalise quaternion
//   recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//   q0 *= recipNorm;
//   q1 *= recipNorm;
//   q2 *= recipNorm;
//   q3 *= recipNorm;
//   Gy95t.orientation.w = q0;
//   Gy95t.orientation.x = q1;
//   Gy95t.orientation.y = q2;
//   Gy95t.orientation.z = q3;    
// }

volatile float twoKp = 1.0f;     // 2 * proportional gain (Kp)
volatile float twoKi = 0.003f;     // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  float yaw;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = 1 / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;      
    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    //误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    // 计算并应用积分反馈（如果启用）
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / SAMPLING_FREQ);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / SAMPLING_FREQ);
      integralFBz += twoKi * halfez * (1.0f / SAMPLING_FREQ);
      gx += integralFBx;        // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f;       // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }
    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / SAMPLING_FREQ));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / SAMPLING_FREQ));
  gz *= (0.5f * (1.0f / SAMPLING_FREQ));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx); 
  // Normalise quaternion
  //recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  recipNorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  Gy95t.orientation.w = q0;
  Gy95t.orientation.x = q1;
  Gy95t.orientation.y = q2;
  Gy95t.orientation.z = q3;    
}
/**
 * @Date: 2022-08-02 17:52:44
 * @Input :  ,Return:
 * @Functonname :Find_String
 * @MethodAuthor: Rolle,robot_luoer@163.com
 * @Description :  txt source data , ch which you find   repro_data  the return data char what deal resource
 * @功能 :输入 寻找字符串数组地址，txt 寻找结束字符， ch  找到的数据存入 repro_dataa   返回寻找到的地址ch char 
*/
char *Find_String(char *txt, char *ch,char *Repro_data)
{
  int index = 0;
  Repro_data[index] = 0;
  char *c;
  while (*txt != '\0')
  {
    for (c = ch; *c != '\0'; c++)
    {
      if (*txt == *c)
      {
        return txt;
      }
    }
    Repro_data[index] = *txt;
    index++;
    Repro_data[index] = 0;
    txt++;
  }
  return txt;
}

/**
 * @Input :  ,Return:
 * @Functonname :
 * @MethodAuthor: Rolle,robot_luoer@163.com
 * @Description : 
 * @功能 :
*/
int Auto_ston(const char *str)
{

	int s=0;
	bool falg=false;
	while(*str==' ')
	{
		str++;
	}
	if(*str=='-'||*str=='+')
	{
		if(*str=='-')
		falg=true;
		str++;
	}
 
	while(*str>='0'&&*str<='9')
	{
		s=s*10+*str-'0';
		str++;
		if(s<0)
		{
			s=2147483647;
			break;
		}
	}
	return s*(falg?-1:1);	
}
/**
** @name:              
** @brief:             adc trans to sho
** @param:             
** @date:              2024-08-20
** @version:           V0.0
---------------------------------------*/
float AdcDataTrans(short data)
{
       static  short _old_data=0,_new_data=0;
      if((_old_data==0)&&(_new_data==0))
      {
         _new_data = data;
      }
      else 
      {_new_data =( 0.6* data) + (0.4 *_old_data);}
      
       _old_data =(_new_data + data )/2;
      float buffer = (_new_data/100.0);

      // return ((buffer*3.33)/SeparatePower);
      return buffer;
}