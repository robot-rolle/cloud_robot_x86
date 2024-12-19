#ifndef _YD_MATH_H_
#define _YD_MATH_H_
#include <vector>
#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <time.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <yd_planner_core.h>
class RandNumber
{
    int  i;
    public:
        RandNumber();
        float Randomize();
};

typedef struct _POINT_
{
        float x ;//point  x
        float y ; //point y
        float z ;//point z

}POINT;

typedef struct _TREENODE_
{
        POINT pos;   // the node  pos of the  plan not in the map
        float value; //buffer_value of the sample point  not global value  
        float global_value,local_value;//global cost and  local cost
        int _parent_index; //the index value of father index 
}TREENODE;
 
float Norm(std::vector<float>, std::vector<float>);

float Norm(TREENODE , TREENODE);


float Sign(float );

std::vector<float> Nearest(std::vector<std::vector<float> >, std::vector<float>);
TREENODE  Nearest(std::vector<TREENODE> , TREENODE );

std::vector<float> Steer(std::vector<float> , std::vector<float>, float);
TREENODE Steer(TREENODE,TREENODE,float);

float  Rewire(std::vector<TREENODE> x);

// std::vector<float> Rewire(std::vector<std::vector<float> >);/
/**
        ** @name:              
        ** @brief:             
        ** @param:             
        ** @date:              2024-08-28
        ** @version:           V0.0
---------------------------------------*/
int gridValue(nav_msgs::OccupancyGrid &, std::vector<float>);
int gridValue(nav_msgs::OccupancyGrid &, TREENODE);

/**
** @name:              
** @brief:             
** @param:             
** @date:              2024-08-29
** @version:           V0.0
---------------------------------------*/
char ObstacleFree(std::vector<float> , std::vector<float> & , nav_msgs::OccupancyGrid);

float Rewire(std::vector<TREENODE> x);

/**
** @name:              ObstacleCheck
** @brief:             
** @param:             
** @date:              2024-08-30
** @version:           V0.0
---------------------------------------*/
char  ObstacleCheck(TREENODE,TREENODE, nav_msgs::OccupancyGrid);


void EigenCount();
#endif