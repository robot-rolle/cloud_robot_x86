#ifndef _GRID_PATHMAP_H_
#define _GRID_PATHMAP_H_
#include <Base_Expander.h>
#include <ros/ros.h>

namespace yd_global_planner
{
    class grid_pathmap: public TraceBackBase
    {
            public:
                 grid_pathmap(PotentialCostCalculator *Pcostcal):TraceBackBase(Pcostcal){}
                 virtual ~grid_pathmap(){}
                 bool GetPath(float* potential, double start_x,double start_y,
                                  double end_x,     double end_y,
                                  std::vector<std::pair<float, float> > &path);
            /**
            ** @name:              
            ** @brief:             
            ** @param:             
            ** @date:              2024-09-06
            ** @version:           V0.0
            ---------------------------------------*/
    };
}



#endif