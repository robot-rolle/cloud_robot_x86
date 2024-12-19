#ifndef _BASE_EXPANDER_H_
#define _BASE_EXPANDER_H_
#include <yd_math.h>
#include <PotentialCost_calculator.h>
#include <math.h>
#include <mtrand.h>
#include <TraceBackBase.h>

namespace yd_global_planner
{
  class Base_Expander
  {
        public:
          Base_Expander(void);
          Base_Expander(PotentialCostCalculator * pcc,int nx,int ny):
          unknow_(true),lethal_cost_(253),neutral_cost_(50),factor_(3.0),
          pcc_(pcc)
          {
               SetSize(nx,ny);
          }
       virtual ~Base_Expander(){}
       virtual void  SetSize(int nx, int ny)
       {
            nx_ = nx;
            ny_ = ny;
            ns_ = nx * ny ;
       }
       protected:
            inline int toIndex(int x, int y)
            {
                        return x + nx_*y;
            }
            int nx_,ny_,ns_;
            bool unknow_;
            unsigned char neutral_cost_,factor_,lethal_cost_;
            int cells_visited;
            PotentialCostCalculator* pcc_;
  };
}

#endif