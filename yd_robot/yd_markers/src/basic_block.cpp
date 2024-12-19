#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "basic_block.h"
#include <cstdlib>
#include "path_plan_rrt.h"      
#include "path_plan_rrt_star.h"
int main(int argc, char** argv)
{
    ros::init(argc,argv,"Path_Algorithm");
    ROS_INFO("start_set_map");
    ros::Time _Now,_Last_Time;
    path_map rand_map;
    RRT_STAR rrts_luoer(4);
    sleep(2);
    ros::Rate rate(10);//发布频率1s2次
    double testtime;
    // rrt_luoer.Task();
    rrts_luoer.Task();
    _Last_Time = ros::Time::now();  
    while(ros::ok())
    {
       _Now = ros::Time::now();	
      testtime =(_Now -_Last_Time).toSec();      //cout<<" time is "<<testtime<<endl;
       rrts_luoer.RRTStar();
      rate.sleep();
      ros::spinOnce();
      _Last_Time = _Now; //Record the time and use it to calculate the time interval //记录时间，用于计算时间间隔       
    }
    ros::spin();
    ROS_INFO("end");
    return 0;
}

/**
 * @Input : start_p endp,color 1 red 2 green 3 blue  ,Return:void
 * @Functonname :Add_Line
 * @MethodAuthor: Rolle,robot_luoer@163.com
 * @Description : 
 * @功能 :画线函数 起点 终点 颜色设定 1 红 2 绿 3 蓝  scale_size 线粗细(0-1)  height 高度
*/
void path_map::Add_Line(POINT& start_p,POINT& end_p,int color,float scale_size = 0.2,float height = 0.0)
{
    //Clear_Lines();
    static int count = 0;
    lines.header.frame_id="map";
    lines.header.stamp=ros::Time::now();
    lines.ns="line_mark";
    lines.id=count;
    lines.type =visualization_msgs::Marker::LINE_LIST;
          lines.pose.orientation.x = 0.0;
          lines.pose.orientation.y = 0.0;
          lines.pose.orientation.z = 0.0;
          lines.pose.orientation.w = 1.0;
          lines.scale.x = scale_size;
          lines.scale.y = scale_size;
        if(color==1)
          {lines.color.r =1.0; lines.color.g =0.0;lines.color.b =0.0;}
          else if(color==2)
          {lines.color.r =0.0; lines.color.g =1.0;lines.color.b =0.0;}
          else
           {lines.color.r =0.0; lines.color.g =0.0;lines.color.b =1.0;}
          lines.color.a =1.0;        
          geometry_msgs::Point p_start;
          p_start.x=start_p.x;
          p_start.y=start_p.y;
          p_start.z =height;
          lines.points.push_back(p_start);
          geometry_msgs::Point p_end;
          p_end.x =end_p.x;
          p_end.y =end_p.y;
          p_end.z = height;
          lines.points.push_back(p_end);

          count++;
          Lines_Array.markers.push_back(lines);    
          line_publisher.publish(Lines_Array);
          lines.points.pop_back();
          lines.points.pop_back();
          lines.points.clear();       
          Lines_Array.markers.clear();
}
/** warnning not new the new map data
 * @Input :vector<MARKERS_ALL>   ,Return:void
 * @Functonname :
 * @MethodAuthor: Rolle
 * @Description :  add rand block
 * @功能 :添加随机障碍物 动态的
*/
void path_map::Add_Block(MARKERS_ALL markers)
{
  int d_x,d_y;
  for(int i =0; i<6;i++) // set three block rand
  {
  markers.m_cube.header.frame_id ="map";   
  markers.m_cube.header.stamp  = ros::Time::now();
  markers.m_cube.ns ="block_map";
  markers.m_cube.id = i;

  markers.m_cube.type = visualization_msgs::Marker::CUBE;
  markers.m_cube.action = visualization_msgs::Marker::ADD;

  // d_x=Rand_Twenty-10;d_y=Rand_Twenty-10;
  d_x=Randstart_end(20,40);d_y=Randstart_end(20,40);
  while(Map_origin.data_map[d_x][d_y]==1)  //地图检测
  { 
  d_x=Randstart_end(20,40);d_y=Randstart_end(20,40);

  }
  markers.m_cube.pose.position.x=d_x;
  markers.m_cube.pose.position.y=d_y;
  markers.m_cube.pose.position.z = 0.1;

  markers.m_cube.pose.orientation.x = 0;
  markers.m_cube.pose.orientation.y = 0;
  markers.m_cube.pose.orientation.z = 0;  
  markers.m_cube.pose.orientation.w = 1;

  markers.m_cube.scale.y =1.0;
  markers.m_cube.scale.x =1.0;
  markers.m_cube.scale.z = 0.01;

  markers.m_cube.color.r=1.0f;
  markers.m_cube.color.g=0.0f;
  markers.m_cube.color.b=0.0f;
  markers.m_cube.color.a = 1.0f;

  markers.marry_cube.markers.push_back(markers.m_cube);
  }
   block_publihser.publish(markers.marry_cube);
   markers.marry_cube.markers.clear();
}
/**
 * @Input :vector<MARKERS_ALL>   ,Return:void
 * @Functonname :
 * @MethodAuthor: Rolle
 * @Description : 
 * @功能 :添加随机障碍物 静态的
*/
void path_map::Obstacle_Block(MARKERS_ALL markers)
{
  for(int i =0; i<Static_Obstacle.size();i++)
  {
  markers.m_cube.header.frame_id ="map";
  markers.m_cube.header.stamp  = ros::Time::now();
  markers.m_cube.ns ="obs_map";
  markers.m_cube.id = i;
  markers.m_cube.type = visualization_msgs::Marker::CUBE;
  markers.m_cube.action = visualization_msgs::Marker::ADD;

  markers.m_cube.pose.position.x = Static_Obstacle[i].x;
  markers.m_cube.pose.position.y = Static_Obstacle[i].y;
  markers.m_cube.pose.position.z =2.0;

  markers.m_cube.pose.orientation.x =1;
  markers.m_cube.pose.orientation.y = 0;
  markers.m_cube.pose.orientation.z =0;  
  markers.m_cube.pose.orientation.w = 0;
  if(i==0){
      markers.m_cube.scale.y =1.0;
      markers.m_cube.scale.x =1.0;
      markers.m_cube.scale.z = 0.01;
      markers.m_cube.color.r=0.0f;
      markers.m_cube.color.g=0;
      markers.m_cube.color.b=1.0f;}
  else if(i==1){ //second point 50 50{
      markers.m_cube.scale.y =1.0;
      markers.m_cube.scale.x =1.0;
      markers.m_cube.scale.z = 0.01;
      markers.m_cube.color.r=1.0f;
      markers.m_cube.color.g=0.0f;
      markers.m_cube.color.b=0.0f;}
  else{
     if(Static_Obstacle[i].row_or_column){
      markers.m_cube.scale.x =cube_s;
      markers.m_cube.scale.y =cube_l;}
      else{
      markers.m_cube.scale.x =cube_l;
      markers.m_cube.scale.y =cube_s;
      }
      markers.m_cube.scale.z = 5;
      markers.m_cube.color.r=1.0f;
      markers.m_cube.color.g=1.0f;
      markers.m_cube.color.b=1.0f;}
  markers.m_cube.color.a = 1.0f;
  markers.marry_cube.markers.push_back(markers.m_cube);
  }
  obstacle_publisher.publish(markers.marry_cube);
  markers.marry_cube.markers.clear();
}


/**
 * @Input : start point , end point  step_size ,Return:bool ture or false
 * @Functonname :
 * @MethodAuthor: Rolle
 * @Description : check the path is vaild
 * @功能 :检查起点和终点 2点之间是否有障碍物 第三个参数是 2点之间的直线分段数
*/
bool path_map::Check_Path(POINT& p_,POINT& p_t,unsigned int stepsize_check)//bug3
{
  
  int dx,dy;
  float Rad_P,step;
  float length=sqrt(pow((p_t.x-p_.x),2)+pow((p_t.y-p_.y),2));

  step =(float)length/stepsize_check;
  cout<<" step_size "<<step<<endl;

  Rad_P =atan2((p_t.y-p_.y),(p_t.x-p_.x)); //得到弧度 角度 -pi 到 pi
  
  for(int i =1;i<=stepsize_check;i++)
  {
      dx=(p_.x+(step*i)*cos(Rad_P));
      dy=(p_.y+(step*i)*sin(Rad_P));
      cout<<" dx: "<<dx<<" "<<dy<<endl;
      if(Map_origin.data_map[dx][dy]==1)
      { 
        cout<<"_________error point"<<endl;
        return  false;
      }
      else ;
  }
  return true;
}



POINT path_map::Sample_Point()
{
    POINT _new_p,end_point;         //新的
    int d_x,d_y;
    end_point.x=Static_Obstacle[1].x;
    end_point.y=Static_Obstacle[1].y;
    end_point.z=0.1;

    srand((unsigned int )time(NULL)); //reset the link of the srand
    d_x=Randstart_end(row_column,0);
    d_y=Randstart_end(row_column,0);
    while(Map_origin.data_map[d_x][d_y]==1)
   {
       d_x=Randstart_end(row_column,0);
       d_y=Randstart_end(row_column,0);
   }
   _new_p.x=(d_x);
   _new_p.y=(d_y);
   _new_p.z=0.1;
   _new_p.value=0;
  if(Rand_zero>0.6) {return end_point ;  }
  else 
    return _new_p;
}



/**
 * @Input :  ,Return:
 * @Functonname :
 * @MethodAuthor: Rolle
 * @Description : 
 * @功能 :测距函数二维
*/
float path_map::Distance(POINT& a,POINT& b)
{
  return sqrt(pow((a.x-b.x),2)+pow((a.y-b.y),2));
}

void path_map::Task()
{
  srand((unsigned int )time(NULL)); //reset the link of the srand
  Obstacle_Block(Test_Markers_ob);
  //Add_Block(Test_Markers_ab);
}
void path_map::Build_Map(unsigned int map_size,int border)
{
    int d_x,d_y,dx_min,dy_min;
     vector<vector<int> > obj(map_size, vector<int>(map_size)); //定义二维动态数组5行6列
    srand((unsigned int )time(NULL)); //reset the link of the srand
    for(int i=0; i<10; i++)
    {
      if(i==0){OBSTACLE_T.x=Randstart_end(5,0);OBSTACLE_T.y=Randstart_end(5,0);OBSTACLE_T.row_or_column=0;} //
      else if(i==1){OBSTACLE_T.x=Randstart_end(5,row_column-5);OBSTACLE_T.y=Randstart_end(5,row_column-5);OBSTACLE_T.row_or_column=0;}
      else
      {  //input the rand cube and add offset with x y
          OBSTACLE_T.x=Randstart_end((row_column-cube_l),(cube_l/2)); //随机生成 xy   还没有优化border
          OBSTACLE_T.y=Randstart_end((row_column-cube_l),(cube_l/2));         
          OBSTACLE_T.row_or_column=Rand_ture_false;   //随机生成 横竖
      } //rand the data
      d_x=OBSTACLE_T.x;  //存入 坐标到地图
      d_y=OBSTACLE_T.y;
      if(OBSTACLE_T.row_or_column) //判断是 横竖  如果是1 那么是横的障碍物
      {
          dx_min = d_x-(cube_s/2)-border; //找到最低的坐标 加入边界
          dy_min = d_y-(cube_l/2)-border;
          for(int j=0; j<=(cube_s+2*border);j++) //循环找到每一根竖直的障碍物 
          {
            for (int k= 0; k<=(cube_l+2*border); k++) //循环存入每根竖直的障碍条的点
            {Map_origin.data_map[dx_min+j][dy_min+k]=1;} //存入障碍点
          }          
      }
      else if(i>1) //判断是横条
      {
          dx_min =d_x-(cube_l/2)-border;
          dy_min =d_y-(cube_s/2)-border;
          for(int j=0; j<=(cube_s+2*border);j++)
          {
            for (int k= 0; k<=(cube_l+2*border); k++)
            {Map_origin.data_map[dx_min+k][dy_min+j]=1;}
          }            
      }
      Map_origin.data_map[d_x][d_y]=1; //存入中心点
      Static_Obstacle.push_back(OBSTACLE_T); //存入障碍物     
    }
}
path_map::path_map()
{
  memset(&Test_Markers_ob,0,sizeof(Test_Markers_ob));
  memset(&Test_Markers_ab,0,sizeof(Test_Markers_ab));
  memset(&Map_origin,0,sizeof(Map_origin));
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  row_column=100;
  Build_Map(100,2);
	// private_nh.param<std::string>("map",map_id,"map"); //map 坐标系
  obstacle_publisher = n.advertise<visualization_msgs::MarkerArray>("obstacle_map",10); //发布话题 obstacle
  //rand point  which from rand function
  point_publisher = n.advertise<visualization_msgs::MarkerArray>("point_map",10); //发布话题 point
  //to link the point with the rand point
  line_publisher = n.advertise<visualization_msgs::MarkerArray>("line_map",1000); //发布话题 lines
  block_publihser = n.advertise<visualization_msgs::MarkerArray>("block_map", 10); //send  publisher  block
  start_end_publisher = n.advertise<visualization_msgs::MarkerArray>("start_end_map",10); //发布话题 point

}
path_map::path_map( unsigned int size)
{
  memset(&Test_Markers_ob,0,sizeof(Test_Markers_ob));
  memset(&Test_Markers_ab,0,sizeof(Test_Markers_ab));
  memset(&Map_origin,0,sizeof(Map_origin));
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  row_column=size;
  Build_Map(100,2);
	// private_nh.param<std::string>("map",map_id,"map"); //map 坐标系
  obstacle_publisher = n.advertise<visualization_msgs::MarkerArray>("obstacle_map",10); //发布话题 obstacle
  //rand point  which from rand function
  point_publisher = n.advertise<visualization_msgs::MarkerArray>("point_map",10); //发布话题 point
  //to link the point with the rand point
  line_publisher = n.advertise<visualization_msgs::MarkerArray>("line_map",1000); //发布话题 lines
  block_publihser = n.advertise<visualization_msgs::MarkerArray>("block_map", 10); //send  publisher  block
  start_end_publisher = n.advertise<visualization_msgs::MarkerArray>("start_end_map",10); //发布话题 point

}
 path_map::~path_map()
{
  Static_Obstacle.clear();
  vector<OBSTACLE>().swap(Static_Obstacle);
  ROS_INFO_STREAM("Shutting down");	
}
