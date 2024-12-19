
#ifndef _YD_PLANNER_CORE_H_
#define _YD_PLANNER_CORE_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/voxel_layer.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Point.h>
#include <global_planner/GlobalPlannerConfig.h>
#include <nav_msgs/GetPlan.h>
#include <vector>

namespace yd_global_planner
{

    class  Base_Expander;                   //基础的膨胀类
    class  Grid_Path;                       //存入梯度下降路径

    class YdPlanner : public nav_core::BaseGlobalPlanner
    {
         public:
            //   none input
            YdPlanner();
            //  _name   costmap_ros
            YdPlanner(std::string _name, costmap_2d::Costmap2DROS* costmap_ros);
            //
            YdPlanner(std::string _name, costmap_2d::Costmap2DROS* costmap_ros,\
                      std::string frame_id, costmap_2d::Costmap2D* costmap);

            ~YdPlanner();
                
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros,\
                            std::string frame_id, costmap_2d::Costmap2D* costmap);
            void initialize(void);
            /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose 
         * @param goal The goal pose 
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
            bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);
            /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose 
         * @param goal The goal pose 
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */    
            bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan,
                        double& cost)
            {
                cost = 0;
                return makePlan(start, goal, plan);
            }
            bool computePotential(const geometry_msgs::Point& world_point);
            
            bool getPlanFromPotential(double start_x,\
                                      double start_y,\
                                      double end_x,\
                                      double end_y,\
                                      const geometry_msgs::PoseStamped& goal,\
                                      std::vector<geometry_msgs::PoseStamped>& plan);
            bool validPointPotential(const geometry_msgs::Point &world_point);
            bool validPointPotential(const geometry_msgs::Point &world_point, double tolerance);
            double getPointPotential(const geometry_msgs::Point &world_point);
            void  publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
            /**
            ** @name:              makePlanService
            ** @brief:             对于nav_msgsGetPlan使用 
            ** @param:             
            ** @date:              2024-09-04
            ** @version:           V0.0
            ---------------------------------------*/
            bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
        private:
            costmap_2d::Costmap2DROS* costmap_ros_;    //定义一个costmap 2d ros控制器 订阅和处理消息
            double step_size_, min_dist_from_robot_;
            base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
            boost::mutex lockmutex_;    //定一个互斥锁
            ros::ServiceServer  make_Gplan_srv_;                //定一个服务
            void mapToWorld(double mx, double my, double& wx, double& wy);
            bool worldToMap(double wx, double wy, double& mx, double& my );
            void ClearRobotCell(const geometry_msgs::PoseStamped& global_pose, 
                                 unsigned int mx, unsigned int my);
            void publishPotential(float *potential);
            /**
             * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
             * @param x_i The x position of the robot 
             * @param y_i The y position of the robot 
             * @param theta_i The orientation of the robot
             * @return 
             */
            double footprintCost(double x_i, double y_i, double theta_i);
            int _nx, _ny;
            float convert_offset_;
            float *potential_array_;

        protected:
             std::string frame_id_;
             ros::Publisher plan_pub_;
             costmap_2d::Costmap2D* costmap_;
             bool initialized_, allow_unknow_;
             ros::Time  NewTime;
             ros::Time  OldTime;
    };
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
        // class RRTExpansion {
        //         public:
        //                 RRTExpansion(PotentialCalculator* p_calc, int nx, int ny);
        //                 virtual ~RRTExpansion() {}
        //                 bool calculatePotentials(unsigned char* costs, //重构 计算潜力
        //                                          double start_x, 
        //                                          double start_y, 
        //                                          double end_x, 
        //                                          double end_y, 
        //                                          int cycles,
        //                                          float* potential);
        //                 double Distance(POINT& _start, POINT& _end);
        //                 bool   CheckPath(POINT& _start, POINT _end, double stepsize);                     
        //         private:
        //                 void Extend();
        //                 POINT DistanceCost(std::vector<Index> &Tree, POINT &point);
        //                 std::vector<Index> Path_;
        // };

/**
       * @brief Given a goal pose in the world, compute a plan
       * @return True if a valid plan was found, false otherwise
*/
};
#endif