#include "functions.h"


// rdm class, for gentaring random flot numbers
rdm::rdm() {i=time(0);} 
float rdm::randomize() 
{ 
  i=i+1;  
  srand (i); 
  return float(rand())/float(RAND_MAX);
}



//Norm function 
float Norm(std::vector<float> x1,std::vector<float> x2)
{
  return pow(	(pow((x2[0]-x1[0]),2)+pow((x2[1]-x1[1]),2))	,0.5);
}
//sign function
float sign(float n)
{
  if (n<0.0){return -1.0;}
  else{return 1.0;}
}
//Nearest function
std::vector<float> Nearest(  std::vector< std::vector<float>  > V, std::vector<float>  x)
{
  float min=Norm(V[0],x); //input the first  point length
  int min_index;
  float temp;
    for (int j=0;j<V.size();j++)
    {
      temp=Norm(V[j],x);
      if (temp<=min){
      min=temp;
      min_index=j;}
    }
      return V[min_index];
}

/**
** @name:              
** @brief:              xnearest  and xrand not stepsize set
** @param:              x_nearest 最近的点  x_rand 采样点 stepsize 步长
** @date:              2024-08-26
** @version:           V0.0
---------------------------------------*/
//Steer function
  std::vector<float> Steer(  std::vector<float> x_nearest , std::vector<float> x_rand, float stepsize)
  {
    std::vector<float> x_new;
    if(Norm(x_nearest,x_rand)<=stepsize) 
    {
      x_new = x_rand;
    }
    else
    {
      if(x_rand[0]!=x_nearest[0])   //判断分母是否为0 确定delta x 的误差大小   
      {
          float  Rad_P = atan2((x_rand[1]-x_nearest[1]),(x_rand[0]-x_nearest[0]));  //获得tan角度
            x_new.push_back(x_nearest[0] + (stepsize*cos(Rad_P)));                 //存入x cos角度
            x_new.push_back(x_nearest[1] + (stepsize*sin(Rad_P)));                 //存入y sin角度
      }
      else if(x_rand[0]==x_nearest[0])                                        //如果是垂直点 直接存入
      {
        x_new[0]=x_nearest[0];
        x_new[1]=x_nearest[1]+stepsize;
      }
    }
       return x_new;    
}                           
  //gridValue function
  int gridValue(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp)
  {   
    float resolution = mapData.info.resolution;
    float Xstartx = mapData.info.origin.position.x;
    float Xstarty = mapData.info.origin.position.y;
    float width = mapData.info.width;
    std::vector<int8_t> Data = mapData.data;
    //returns grid value at "Xp" location
    //map data:  100 occupied      -1 unknown       0 free
    float indx=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) );
    int out; out=Data[int(indx)];
        return out;
  }



// ObstacleFree function------------------------------------- check the path of the value
// -1  unk   0  obs  1  free
char ObstacleFree(std::vector<float> xnear, std::vector<float> &xnew, nav_msgs::OccupancyGrid mapsub)
{
      float rez=float(mapsub.info.resolution)*.2; //get mini  cell
      float stepz=int(ceil(Norm(xnew,xnear))/rez);  
      std::vector<float> xi=xnear;
      char  obs=0; char unk=0;
      // geometry_msgs::Point p;
      for (int c=0;c<stepz;c++)
      {
        xi=Steer(xi,xnew,rez);  //  every step add to the point which  we want
        if(gridValue(mapsub,xi) ==100) 
        {     obs=1; }
        if(gridValue(mapsub,xi) ==-1)
        {      unk=1;	break;}
      }
        char out=0;
        xnew   =xi;   //重新复制新点  作为存入点
        out =  unk==1 ? -1 : obs==1 ? 0:1;  //多元判断 这个点是啥 返回 -1 0 1状态
          return out;    
 }
 

     
   


  
 
 
 
 





























