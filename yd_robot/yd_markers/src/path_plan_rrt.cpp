#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "basic_block.h"
#include <cstdlib>
#include "path_plan_rrt.h"
#include <tf/transform_broadcaster.h>

/**
 * @Input :  ,Return:
 * @Functonname :
 * @MethodAuthor: Rolle
 * @Description : 
 * @功能 :随机采样函数
*/

void RRT_PLAN::Get_Path(vector<POINT>& path)
{
  static int B_Index=path.back()._parent_index;
  while(B_Index>0)
  {
    B_Index=path[B_Index]._parent_index;
     Add_Line(path[B_Index],path[path[B_Index]._parent_index],1,0.3,1.0); 
    cout<<" the last index "<<path[B_Index]._parent_index<<endl;
  }
}      

/**
 * @Input :  ,Return:
 * @Functonname :
 * @MethodAuthor: Rolle
 * @Description : RRT
 * @功能 : 
*/
void RRT_PLAN::RRT(void){  
  POINT P_Near=distanceCost(Path_Tree,end_);
   if(Distance(P_Near,end_)<Step_length)
   { 
     Flag_State=true;
     Add_Line(P_Near,end_,2,0.2,0.0);

     end_._parent_index= index_parents;
     Path_Tree.push_back(end_);
     Map_origin.data_map[end_.x][end_.y]=1;
   }else;
  if(Flag_State!=true)
  {
    Extend();
  }
  else
  {
    Get_Path(Path_Tree);
    //ROS_INFO("find  the final path__________read set up it ");
  }

  }
  /**
   * @Input : none ,Return:none
   * @Functonname :Extend
   * @MethodAuthor: Rolle
   * @Description : extend the path and check the path
   * @功能 :扩展函数，地图采样，检查路径和判断路径是否在地图不碰撞。不碰撞 画线
  */
void RRT_PLAN::Extend()          
{
  POINT Q_nearest; //
  float Rad_P;           //insert the radius
  bool state_check;
  //Add_Block(Test_Markers_ab); 加入动态优化
  Q_Rand_ = Sample_Point();   //Q_Rand_ 存入采样点
  Q_nearest=distanceCost(Path_Tree,Q_Rand_);      //get the min point  q_nearest

  Rad_P=atan2((Q_Rand_.y-Q_nearest.y),(Q_Rand_.x-Q_nearest.x));//bug2
  Q_New_.x= Q_nearest.x+(Step_length*cos(Rad_P));
  Q_New_.y=Q_nearest.y+(Step_length*sin(Rad_P));//判断new是否合格

 if(Check_Path(Q_nearest,Q_New_,2)&&(Distance(Q_nearest,Q_Rand_)>Step_length))//最近取样点 符合存入树中
  {
//Add_Line(Q_nearest,Q_Rand_,3); 
    Add_Line(Q_nearest,Q_New_,2,0.2,0);//画线
    Q_New_._parent_index=index_parents;//存入父亲节点地址
    
    Path_Tree.push_back(Q_New_);//存入新点到树中
    Map_origin.data_map[Q_New_.x][Q_New_.y]=1;//在地图中添加节点位置
   }
   else if((Distance(Q_nearest,Q_Rand_)<Step_length)&&(Check_Path(Q_nearest,Q_Rand_,1)))//如果路径小于补偿。补偿又是没有障碍物存入
   {
      Add_Line(Q_nearest,Q_Rand_,2,0.2,0);//画线
      Q_Rand_._parent_index=index_parents;//存入父亲节点地址

      Path_Tree.push_back(Q_Rand_);//存入新点到树中
      Map_origin.data_map[Q_Rand_.x][Q_Rand_.y]=1;//在地图中添加节点位置
   }
}

//get the distance of the tree
/**
 * @Input :  ,Return:
 * @Functonname :
 * @MethodAuthor: Rolle
 * @Description : 
 * @功能 :  随机树， 和采样点  返回最近点 ---->使用后要及时提取index_parents 否则容易 地址错乱
*/
POINT RRT_PLAN::distanceCost(vector<POINT>& path_rrt,POINT& point)  //bug1
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
      index_parents= min_index;//获得父亲节点信息  
      return path_rrt[min_index]; //find the nearest point
}



RRT_PLAN::RRT_PLAN()
{
    start_.x =Static_Obstacle[0].x;
    start_.y =Static_Obstacle[0].y;
    start_._parent_index=0;
    end_.x =Static_Obstacle[1].x;
    end_.y =Static_Obstacle[1].y;// get the ob
    Flag_State=false;
    Path_Tree.push_back(start_);
}
RRT_PLAN::RRT_PLAN(unsigned int step_size,unsigned int map_size)
{
    start_.x =Static_Obstacle[0].x;
    start_.y =Static_Obstacle[0].y;
    start_._parent_index=0;
    start_.global_value=0;
    start_.local_value=0;
    

    end_.x =Static_Obstacle[1].x;
    end_.y =Static_Obstacle[1].y;
    Step_length=step_size;
    row_column=map_size;
    Flag_State=false;
    Path_Tree.push_back(start_);
    cout<<"start_x  "<<start_.x<<"  y "<<start_.y<<endl;

    
}

RRT_PLAN::~RRT_PLAN()
{
 Path_Tree.pop_back();
}