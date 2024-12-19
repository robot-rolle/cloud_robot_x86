#include <global_planner/astar.h>
#include <costmap_2d/cost_values.h>
#include <global_planner/rrt_expand.h>
namespace global_planner
{

        RRTExpansion::RRTExpansion(PotentialCalculator* p_calc, int nx, int ny):
        Expander(p_calc, nx, ny){}
         
        double RRTExpansion::Distance(POINT& _start, POINT& _end)
        {
            return  sqrt( pow(abs(_start._x - _end._x),2) + pow(abs(_start._y - _end._y),2));
        }
              
               
}