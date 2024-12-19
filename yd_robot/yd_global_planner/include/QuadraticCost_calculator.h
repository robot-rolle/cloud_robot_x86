#ifndef QUADRATICCOST_CALCULATOR_H_
#define QUADRATICCOST_CALCULATOR_H_
#include <PotentialCost_calculator.h>
#include <vector>
#include <yd_planner_core.h>


/**
** @name:              
** @brief:            
 1.计算“一个点”的可行性 —— p_calc_：PotentialCalculator::calculatePotential()、 QuadraticCalculator::calculatePotential()
2.计算“所有”的可行点 —— planner_：DijkstraExpansion::calculatePotentials()、 AStarExpansion::calculatePotentials()
3.从可行点中“提取路径” —— path_maker_：GridPath::getPath()、 GradientPath::getPath()
** @param:             
** @date:              2024-09-12
** @version:           V0.0
---------------------------------------*/
namespace yd_global_planner
{
    class QuadraticCostCalculator : public  PotentialCostCalculator
    {
            public:
                QuadraticCostCalculator(){}
                QuadraticCostCalculator(int x,int y) :
                PotentialCostCalculator(x,y){}
                ~QuadraticCostCalculator()
                {}

                float CalculatePotentialCost(float *potential,
                                                    unsigned char cost,
                                                    int  n,
                                                    float prev_potential );
    };

    
}

#endif
