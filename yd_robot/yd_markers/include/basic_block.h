#ifndef _BASIC_BLOCK_H_
#define _BASIC_BLOCK_H_
//http://wiki.ros.org/visualization_msgs

#include "ros/ros.h"
#include <bits/stdc++.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <string> 
#include <cstdlib>
#include <time.h>
#include <stdio.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
using namespace std;
//不移动两个地图信息
// 找到终点
// 路径检测  
// 优化节点
// 分离地图


// 分化功能-->地图信息       ---->路径信息

//             实时更新同步--->节点信息---->算法检测(地图1)------> 存入地图2----> 算法检测(地图1+2)
//             ----------循环到找到 删除2 提取路径
//   rand函数不是真随机函数，依赖srand 函数 



#define  Rand_zero   ((rand()%10)/10.0f)
#define  Rand_u       ((rand()%2)>0?(1):-1)   // retrun -1 or 1
#define  Rand_float_u  (((rand()%20)/10.0f)-1)  //return  -1.0f - 1.0f
#define  Rand_ture_false  (rand()%2)

//data_length  data_Start_point
#define  Randstart_end(data_len,data_dx)  ((rand()%data_len)+data_dx)   //data_len length  data_dx  data of move dx




#define  cube_s     4
#define  cube_l     20
#define  cube_h     2

typedef  struct _POINT_
{
    int x; //point x
    int y; //point y
    int z; // point z
    float value; //buffer_value of the sample point  not global value  
    float global_value,local_value;//global cost and  local cost
    int _parent_index; //the index value of father index 
}POINT;//节点信息

typedef struct MAP_VISUAL
{

    int  data_map[100][100]={0};

    POINT point;
    vector<POINT> point_x;
    
        
}MAP_VISUAL;//地图信息
typedef  struct _OBSTACLE_
{
    int x ;  //the pose of x
    int y ;  //the pose of y
    int z  = 0;  //the pose of z
    int border_x; // border of the x
    int border_y; // border of the y
    int border_z; // border of the z
    int row_or_column;
}OBSTACLE;//地图上障碍物结构体
typedef struct _MARKERS_ALL_
{
    visualization_msgs::Marker m_cube;
    visualization_msgs::MarkerArray marry_cube;
    POINT   scale,pose;
    OBSTACLE obstacle;
}MARKERS_ALL; //画画结构体 

class path_map
{
    public:
        path_map();
        
        path_map(unsigned int row );
        ~path_map();
        void Setmap();
        void Task();
        unsigned int row_column;
        ros::Publisher obstacle_publisher,point_publisher,line_publisher,start_end_publisher;
        ros::Publisher block_publihser ;
        visualization_msgs::Marker lines,obs_block;
        visualization_msgs::MarkerArray Obs_Array, Point_Array,Lines_Array;
        vector<OBSTACLE> Static_Obstacle; //
        void Obstacle_Block(MARKERS_ALL markers);
        void Add_Block(MARKERS_ALL markers);               
        void Build_Map(unsigned int mpa_size,int border);
        void Add_Line(POINT& start_p,POINT& end_p,int color,float scale_size,float height);//画线函数
        MARKERS_ALL Test_Markers_ob,Test_Markers_ab,Test_Markers_lines;
        OBSTACLE  OBSTACLE_T;
        MAP_VISUAL Map_origin;
       
        POINT Sample_Point();
        float Distance(POINT& a,POINT& b); //2点之间的距离 返回距离 2维
        bool  Check_Path(POINT& start_,POINT& end_, unsigned int stepsize_check);//检查起点和终点 插值后的点是否符合           


    private:
        unsigned int x_size,y_size;

        
};
#endif

