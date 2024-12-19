#ifndef _POTENTIALCOST_CALCULATOR_H_
#define _POTENTIALCOST_CALCULATOR_H_

#include<algorithm>

namespace yd_global_planner
{
    /**
    ** @name:              PotentialCostCalculator
    ** @brief:             
    ** @param:              一个计算潜在代价地图大小的基类 四领域查找
    ** @date:              2024-09-04
    ** @version:           V0.0
    ---------------------------------------*/
    class PotentialCostCalculator
    {
        public:
                PotentialCostCalculator(int nx, int ny)
                {
                    SetSize(nx,ny);
                }
                virtual ~PotentialCostCalculator(){}
                /**
                ** @name:              
                ** @brief:             // get min of neighbors 四邻域 找最小值  存入最小
                ** @param:                  返回最小代价
                ** @date:              2024-09-03
                ** @version:           V0.0
                ---------------------------------------*/
                virtual float CalculatePotentialCost(float *potential,
                                                    unsigned char cost,
                                                    int  n,
                                                    float prev_potential=-1)
                {
                    if(prev_potential < 0)
                    {
                        float min_h = std::min( potential[n -1], potential[n + 1]),
                              min_v = std::min( potential[n -nx_], potential[n + nx_]);
                        prev_potential = std::min(min_h,min_v);
                    }
                    return prev_potential + cost;
                }
                
        private :
                /**
                ** @name:            SetSize  
                ** @brief:           reset the size of the map
                ** @param:             
                ** @date:              2024-09-03
                ** @version:           V0.0
                ---------------------------------------*/
                virtual  void SetSize(int nx, int ny)
                {
                    nx_ = nx;
                    ny_ = ny;
                    ns_ = nx * ny;
                }
                /**
                ** @name:              
                ** @brief:             3d
                ** @param:             
                ** @date:              2024-09-03
                ** @version:           V0.0
                ---------------------------------------*/
                virtual void  SetSize(int nx, int ny, int nz)
                {
                    nx_ = nx;
                    ny_ = ny;
                    nz_ = nz;
                    ns_ = nx * ny;
                }
        protected:
            /**
            ** @name:              
            ** @brief:             return  the index of the map which pos 
            ** @param:             
            ** @date:              2024-09-03
            ** @version:           V0.0
            ---------------------------------------*/
            inline int toIndex(int x, int y)
            {   
                return  x+nx_*y;
            }
            inline int toIndex(int x, int y, int z)
            {
                return  x+ (nx_*y) +( nx_*ny_ *z);
            }
                int nx_,ny_,nz_,ns_;

    };




}
#endif