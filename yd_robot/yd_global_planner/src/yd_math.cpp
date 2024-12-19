



#include "yd_math.h"
#include <cmath>


float Sign(float x)
{
    return  x>=0? 1.0 :-1.0;
}

std::vector<float> Nearest(std::vector<std::vector<float>> V, 
                                std::vector<float> x)
{
    float min = Norm(V[0],x);
    int min_index;
    float temp;
    for(int j = 0; j <V.size(); j++)
    {
        temp = Norm(V[j],x);
        if(temp<=min)
        {
            min = temp;
            min_index = j;
        }
    }
    return V[min_index];

}
/**
** @name:              Nearest
** @brief:             
** @param:             vector<TREENODE> Tree
** @param              TREENODE     rand_
** @date:              2024-09-03
** @version:           V0.0
---------------------------------------*/
TREENODE  Nearest(std::vector<TREENODE> Tree_ , TREENODE rand_ )
{
    TREENODE  nearest_;
    float min_value= Norm(Tree_[0],rand_);
    int min_index;
    float buffer_value;
    for(int i = 0; i<Tree_.size(); i++)
    {
        buffer_value = Norm(Tree_[i],rand_);
        if(buffer_value<min_value)
        {
             min_index = i;
             min_value= buffer_value;
        }
    }
    return Tree_[min_index];
}
/**
** @name:              
** @brief:             RRTstar rewire
** @param:             
** @date:              2024-09-03
** @version:           V0.0
---------------------------------------*/
float Rewire(std::vector<TREENODE> x)
{
    float y =0;
    return  y;
}

/**
** @name:              Steer
** @brief:             
** @param:             
** @date:              2024-08-23
** @version:           V0.0
---------------------------------------*/
std::vector<float> Steer(std::vector<float> x_nearest,
                         std::vector<float> x_rand,
                         float steplength)
{
    std::vector<float> x_new;
    if(Norm(x_nearest,x_rand)<=steplength)
    {  x_new = x_rand;}
    else 
    {
        if(x_rand[0]!=x_nearest[0])
        {
            float tanTheta =  atan2((x_rand[1] - x_nearest[1]),(x_rand[0]-x_nearest[0]));
            x_new[0] = x_nearest[0]+steplength*cos(tanTheta);
            x_new[1] = x_nearest[1]+steplength*sin(tanTheta);

        }
        else if(x_rand[0]==x_nearest[0])
        {
            x_new[0]=x_nearest[0];
            x_new[1] = x_nearest[1] + steplength; //存入
        }
    }
    return x_new;
}

/**
** @name:              Steer
** @brief:             
** @param:             
** @date:              2024-08-30
** @version:           V0.0
---------------------------------------*/
TREENODE Steer(TREENODE &_nearest,TREENODE &_rand,float stepsize)
{
    TREENODE _new;
    if(Norm(_nearest,_rand)<=stepsize)
    {
        _new = _rand;
    }
    else 
    {
        if(_rand.pos.x != _nearest.pos.x)
        {
            float ThetaTan = atan2((_rand.pos.y-_nearest.pos.y),(_rand.pos.x - _nearest.pos.x));
            _new.pos.x = _nearest.pos.x+stepsize*cos(ThetaTan);
            _new.pos.y = _nearest.pos.y+stepsize*sin(ThetaTan);
        }
        else if(_rand.pos.x = _nearest.pos.x)
        {
            _new.pos.x = _nearest.pos.x;
            _new.pos.y = _nearest.pos.y + stepsize;
        }
    }
    return _new;
}
/**
** @name:              Norm
** @brief:             return the norm length of two point
** @param:             
** @date:              2024-08-23
** @version:           V0.0
---------------------------------------*/
float Norm(std::vector<float> x1, std::vector<float> x2)
{
    return pow( (pow((x2[0]-x1[0]),2)+pow((x2[1]-x1[1]),2)), 0.5);
}

float Norm(TREENODE &start_, TREENODE &end_)
{
    return pow((pow((start_.pos.x-end_.pos.x),2)+pow((start_.pos.y-end_.pos.y),2)),0.5);
}

/**
** @name:              gridValue
** @brief:             
** @param:             
** @date:              2024-08-23
** @version:           V0.0
---------------------------------------*/
int gridValue(nav_msgs::OccupancyGrid &mapdata, std::vector<float> xp)
{
    float   resolution = mapdata.info.resolution;
    float   Xstartx =mapdata.info.origin.position.x;
    float   Xstarty =mapdata.info.origin.position.y;
    float   width = mapdata.info.width;

    std::vector<signed char> Data = mapdata.data;  
    //向下取整 得到坐标 index
    float  indx= (floor((xp[1]-Xstarty)/resolution) * width)+(floor((xp[0]-Xstartx)/resolution)); 

    int out;
    out = Data[int(indx)];
    return out;
}
/**
** @name:              gridValue
** @brief:             get the value of the point which in the map
** @param:             
** @date:              2024-08-30
** @version:           V0.0
---------------------------------------*/
int gridValue(nav_msgs::OccupancyGrid &_mapdata, TREENODE &_nodepose)
{ 
    float resolution = _mapdata.info.resolution;
    float Xstart_ = _mapdata.info.origin.position.x;
    float Ystart_ = _mapdata.info.origin.position.y;

    float width = _mapdata.info.width;
    std::vector<signed char> MapData_ = _mapdata.data;
    float index = (floor((_nodepose.pos.y-Ystart_)/resolution) * width)\
                + (floor(_nodepose.pos.x-Xstart_)/resolution);
    int out = MapData_[int(index)];
    return out;
}

/**
** @name:              
** @brief:             
** @param:             
** @date:              2024-08-26
** @version:           V0.0
---------------------------------------*/

char ObstacleFree(std::vector<float> xnear , std::vector<float> &xnew , 
                    nav_msgs::OccupancyGrid mapsub)
{
    float rez = float(mapsub.info.resolution)*.2;  //rez 步长
    
    
    float stepz = int(ceil(Norm(xnew,xnear))/rez);

    std::vector<float> xi = xnear;
    char obs = 0; char unk =0;

    geometry_msgs::Point p;
    for(int c =0; c<stepz; c++)
    {
        xi = Steer(xi,xnew,rez); //xi 循环步近
        if(gridValue(mapsub,xi)==100)
        {
            obs= 1;
        }
        if(gridValue(mapsub,xi)==-1)
        {
            unk =1 ;
            break;
        }
    }
    char out = 0;
    xnew = xi;
    out = unk==1? -1: obs==1? 0:1;
    return out;
}

void EigenCount()
{
    Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 2);
    std::cout << "Here is the matrix A:\n" << A << std::endl;
    Eigen::VectorXf b_= Eigen::VectorXf::Random(3);
    std::cout<<" the ans sis :\b"<<A.colPivHouseholderQr().solve(b_)<<std::endl;
}