
#ifndef _RRT_EXPAND_H
#define _RRT_EXPAND_H

#include <global_planner/planner_core.h>
#include <global_planner/expander.h>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Point.h>
namespace global_planner {
        /**
        ** @name:              
        ** @brief:             struct POINT
        ** @param:             
        ** @date:              2024-08-14
        ** @version:           V0.0
        ---------------------------------------*/
        typedef  struct _POINT_
        {
                double _x;
                double _y;
                double _z; 
        }POINT;
        /**
        ** @name:              
        ** @brief:            struct POSE
        ** @param:             used for rrt-star or other algorithm
        ** @date:              2024-08-14
        ** @version:           V0.0
        ---------------------------------------*/
        typedef struct _POSE_
        {
                POINT  pose;
                float _value;
                float _global_value, _local_value;  //全局代价信息  局部代价信息
                int _parent_index;                  // 父亲节点坐标
        }POSE;
        /**
        ** @name:              
        ** @brief:             
        ** @param:             
        ** @date:              2024-08-14
        ** @version:           V0.0
        ---------------------------------------*/
        class RRTExpansion : public Expander  {
                public:
                        RRTExpansion(PotentialCalculator* p_calc, int nx, int ny);
                        virtual ~RRTExpansion() {}
                        bool calculatePotentials(unsigned char* costs, //重构 计算潜力
                                                 double start_x, 
                                                 double start_y, 
                                                 double end_x, 
                                                 double end_y, 
                                                 int cycles,
                                                 float* potential);
                        double Distance(POINT& _start, POINT& _end);
                        bool   CheckPath(POINT& _start, POINT _end, double stepsize);                     
                private:
                        void Extend();
                        POINT DistanceCost(std::vector<Index> &Tree, POINT &point);
                        std::vector<Index> Path_;
        };

}
#endif

