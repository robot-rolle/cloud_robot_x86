#ifndef _PATH_PLAN_RRT_STAR_H_
#define _PATH_PLAN_RRT_STAR_H_

#include "ros/ros.h"
#include "basic_block.h"
#include "path_plan_rrt.h"


class RRT_STAR: public path_map 
{
    public :
                 vector<POINT> Origin_Tree,Path_RRT,Path_Range;//origin 全局树   range 局部树  rrt 最终树
                 RRT_STAR();
                 RRT_STAR(int step_size);
                 ~RRT_STAR();
                void  RRTStar(void);
                void  Extend_Star();
                POINT GetNearest(vector<POINT>& path_rrt,POINT& point);// 树 采样点  返回最近点
                void  Re_Write();
                vector<POINT>  GetNear(vector<POINT>& path_rrt,POINT& point,POINT& point_near);
    private :
                void Getrrt_Path(vector<POINT>& path);
                unsigned int step_rrts,range_step,index_old,index_new;
                POINT  Start_P,End_P,Qrand_P,Qnew_P;
                bool  Flag_RRTS;


};


#endif
