#ifndef _TRACEBACKBASE_H_
#define _TRACEBACKBASE_H_
#include <ros/ros.h>
#include <vector>
#include <yd_planner_core.h>
#include <PotentialCost_calculator.h>
namespace  yd_global_planner
{
   class TraceBackBase
    {
        public:
            TraceBackBase(PotentialCostCalculator *PCC_): P_Costc(PCC_){}
             virtual ~TraceBackBase(){}
             virtual bool GetPath(float* potential, double start_x,double start_y,
                                  double end_x,     double end_y,
                                  std::vector<std::pair<float, float> > &path) = 0;
             virtual void SetSize(int xs, int ys)
             {
                xs_ = xs;
                ys_ = ys;
             } 
             inline int GetIndex(int x, int y )
             {
                return  x + xs_* y;
             }
            void setLethalCost(unsigned char lethal_cost)
            {
                    lethal_cost_ = lethal_cost;
            }   
        protected:
            unsigned char lethal_cost_;
            PotentialCostCalculator* P_Costc;
            int xs_,ys_;  //空间大小
    };
}

#endif