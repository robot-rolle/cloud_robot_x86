#ifndef _PATH_PLAN_RRT_H_
#define _PATH_PLAN_RRT_H_

#include "ros/ros.h"
#include "basic_block.h"
#define PI 3.1415926


class RRT_PLAN: public path_map
{
    public :
                vector<POINT> Path_Tree,Node_Path;
                RRT_PLAN(unsigned int step_size,unsigned int map_size);
                RRT_PLAN();
                ~RRT_PLAN(); 
                void RRT();            // 随机树
                void Extend(); //扩展函数
                POINT distanceCost(vector<POINT>& path_rrt,POINT& point);// 树 采样点  返回最近点 

    private :    
                void Get_Path(vector<POINT>& path);
                unsigned int Step_length;
                unsigned int index_parents;
                POINT start_,end_,Q_Rand_,Q_New_;
                bool Flag_State;       
};


#endif
