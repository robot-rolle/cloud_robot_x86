#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "basic_block.h"
#include <cstdlib>
#include "path_plan_rrt.h"
#include "path_plan_rrt_star.h"


void RRT_STAR::Re_Write()
{

}
void RRT_STAR::Extend_Star()          
{
  POINT Q_nearest; //
  float Rad_P;           //insert the radius
  bool state_check;
  //Add_Block(Test_Markers_ab); 加入动态优化
  Qrand_P = Sample_Point();   //Qrand_P 存入采样点
  Q_nearest=GetNearest(Origin_Tree,Qrand_P);      //get the min point  q_nearest
  Rad_P=atan2((Qrand_P.y-Q_nearest.y),(Qrand_P.x-Q_nearest.x));//bug2
  Qnew_P.x= Q_nearest.x+(step_rrts*cos(Rad_P));
  Qnew_P.y=Q_nearest.y+(step_rrts*sin(Rad_P));//判断new是否合格

 if(Check_Path(Q_nearest,Qnew_P,2)&&(Distance(Q_nearest,Qrand_P)>step_rrts))//最近取样点 符合存入树中
  {
//Add_Line(Q_nearest,Qrand_P,3); 
    // if()   //此处添加判断 global函数
    // {

    // }
    Add_Line(Q_nearest,Qnew_P,2,0.2,0);//画线
    Qnew_P._parent_index=index_old;//存入父亲节点地址
    Origin_Tree.push_back(Qnew_P);//存入新点到树中
    Map_origin.data_map[Qnew_P.x][Qnew_P.y]=1;//在地图中添加节点位置
   }
   else if((Distance(Q_nearest,Qrand_P)<step_rrts)&&(Check_Path(Q_nearest,Qrand_P,1)))//如果路径小于补偿。补偿又是没有障碍物存入
   {
      Add_Line(Q_nearest,Qrand_P,2,0.2,0);//画线
      Qrand_P._parent_index=index_old;//存入父亲节点地址

      Origin_Tree.push_back(Qrand_P);//存入新点到树中
      Map_origin.data_map[Qrand_P.x][Qrand_P.y]=1;//在地图中添加节点位置
   }
}



//get the distance of the tree
/**
 * @Input :  ,Return:
 * @Functonname :
 * @MethodAuthor: Rolle
 * @Description : 
 * @功能 :  随机树， 和采样点  返回最近点 ---->使用后要及时提取index_old 否则容易 地址错乱
*/
POINT RRT_STAR::GetNearest(vector<POINT>& path_rrt,POINT& point)  //bug1
{
    float cost=0;
    int min_index;
    cost =path_rrt[0].value=sqrt(pow((path_rrt[0].x-point.x),2)+pow((path_rrt[0].y-point.y),2)); //记录起点和采样点的距离
    min_index=0; //路径的地址
    for(int i=1;i<path_rrt.size();i++)
    {
      path_rrt[i].value=sqrt(pow((path_rrt[i].x-point.x),2)+pow((path_rrt[i].y-point.y),2)); //循环记录路径到采样点距离
      if(path_rrt[i].value<cost){ 
        cost=path_rrt[i].value; //获取最小距离
        min_index= i;//得到地址 index
      }
      else; 
    }
      index_old= min_index;//获得父亲节点信息  不一定是真 
      return path_rrt[min_index]; //find the nearest point
}
/**
 * @Input :  ,Return:返回局部最小代价树 便于后期重新rewrite
 * @Functonname :
 * @MethodAuthor: Rolle
 * @Description : 
 * @功能 :  探索树path_rrt  新的最近点point  返回全局最小代价父亲Point_near
*/
vector<POINT> RRT_STAR::GetNear(vector<POINT>& path_rrt,POINT& point,POINT& point_near)
{
  float cost=0,cost_buffer=0;
  int min_index,k_step;
  vector<POINT> local_tree;
  min_index=index_old;
  cost =path_rrt[0].value=Distance(path_rrt[0],point); //记录起点和采样点的距离
  cost_buffer=path_rrt[index_old].global_value+Distance(path_rrt[index_old],point);
  for(int i=1;i<path_rrt.size();i++)
  {
      path_rrt[i].value=Distance(path_rrt[i],point); //循环记录路径到采样点距离
      if((path_rrt[i].value<range_step)&&Check_Path(path_rrt[i],point,int(path_rrt[i].value/1.2)))//到near点的距离在圈范围内 且路径可达
      { 
         
        local_tree.push_back(path_rrt[i]);//存入到树中 为rewire准备

        if((path_rrt[i].global_value+path_rrt[i].value)<cost_buffer)//全局代价最低替换掉
        {
          min_index =i;
          cost_buffer=path_rrt[i].global_value+path_rrt[i].value;
        }else;
      }
      else; 
  }
  index_new=min_index;//真父亲节点坐标
  point_near=path_rrt[min_index];
  return  local_tree;
}


void  RRT_STAR::RRTStar(void)
{
   POINT P_Near=GetNearest(Origin_Tree,End_P);
   if(Distance(P_Near,End_P)<step_rrts)
   { 
     Flag_RRTS=true;
     Add_Line(P_Near,End_P,2,0.2,0.0);

     End_P._parent_index= index_old;
     Origin_Tree.push_back(End_P);
     Map_origin.data_map[End_P.x][End_P.y]=1;
   }else;
    if(Flag_RRTS!=true)
    {
          Extend_Star();
    }
    else
    {
          Getrrt_Path(Origin_Tree);
    }
}
void RRT_STAR::Getrrt_Path(vector<POINT>& path)
{
  static int B_Index=path.back()._parent_index;
  while(B_Index>0)
  {
    B_Index=path[B_Index]._parent_index;
     Add_Line(path[B_Index],path[path[B_Index]._parent_index],1,0.3,1.0); 
    cout<<" the last index "<<path[B_Index]._parent_index<<endl;
  }
}
RRT_STAR::RRT_STAR()
{
     Start_P.x =Static_Obstacle[0].x;
     Start_P.y =Static_Obstacle[0].y;
    Start_P._parent_index=0;
     End_P.x =Static_Obstacle[1].x;
     End_P.y =Static_Obstacle[1].y;
    Flag_RRTS=false;
    Origin_Tree.push_back(Start_P);
}
RRT_STAR::RRT_STAR(int step_size)
{
    step_rrts=step_size;
    Start_P.x =Static_Obstacle[0].x;
    Start_P.y =Static_Obstacle[0].y;
    Start_P.global_value=0;
    Start_P.local_value=0;
    range_step =step_rrts*3;

    Start_P._parent_index=0;
    End_P.x =Static_Obstacle[1].x;
    End_P.y =Static_Obstacle[1].y;
    Flag_RRTS=false;
    Origin_Tree.push_back(Start_P);
}
RRT_STAR::~RRT_STAR()
{}