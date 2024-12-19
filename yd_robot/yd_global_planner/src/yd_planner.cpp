#include <angles/angles.h>
#include <yd_planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
/*-------------------------------------
** @name:              global plan 重新适配a-star
** @brief:             
** @param:             
** @date:              2024-07-24
** @version:           V0.1
---------------------------------------*/

PLUGINLIB_EXPORT_CLASS(yd_global_planner::YdPlanner, nav_core::BaseGlobalPlanner)


namespace yd_global_planner
{   
  YdPlanner::YdPlanner()
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){}

  YdPlanner::YdPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false)
  {
    initialize(name, costmap_ros);
  }
  YdPlanner::~YdPlanner()
  {
    // deleting a nullptr is a noop
    delete world_model_;
  }
/**
** @name:              initialize
** @brief:             
** @param:             
** @date:              2024-07-26
** @version:           V0.0
---------------------------------------*/
  void YdPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
      if(!initialized_)
      {
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        ros::NodeHandle private_nh( "~/" + name );
        private_nh.param("step_size", step_size_, costmap_->getResolution());
        private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
        world_model_ = new base_local_planner::CostmapModel(*costmap_);
        initialized_ = true;
      }
      else 
        ROS_WARN(" This planner has already been initialized ");
  }
  /**
  ** @name:              footprintCost
  ** @brief:             
  ** @param:             得到机器人合适位置
  ** @date:              2024-07-26
  ** @version:           V0.0
  ---------------------------------------*/
  double YdPlanner::footprintCost(double x_i, double y_i, double theta_i)
  {
      if(!initialized_){
        ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
        return -1.0;
      }
      std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint(); //
      //if we have no footprint... do nothing  空的size  make—plan重设置
      if(footprint.size() < 3)  
        return -1.0;
      //check if the footprint is legal 
      double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
      return footprint_cost;
  }
/**
** @name:              makePlan
** @brief:             
** @param:             算法思路 分2步 point + direction 
      如何拿到 point   使用 势能场+ rrt
      如何拿到 direction   使用二次插值
** @date:              2024-07-26 
** @version:           V0.0
---------------------------------------*/
  bool YdPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, 
      std::vector<geometry_msgs::PoseStamped>& plan)
{

    NewTime=ros::Time::now();
    double DeltaTime=(NewTime-OldTime).toSec();
    ROS_INFO(" the delta time is [%f]",DeltaTime);
    if(!initialized_)
    {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }
    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    plan.clear();
   // costmap_ = costmap_ros_->getCostmap();
    costmap_ = costmap_ros_->getCostmap();   //获取代价地图
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())  //地图id不匹配
    {
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    const double yawS_ = tf2::getYaw(start.pose.orientation); 
    const double yawE_ = tf2::getYaw(goal.pose.orientation);

    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    double diff_x = goal_x - start_x;  //delta 两点距离误差
    double diff_y = goal_y - start_y;  //

    double diff_yaw = angles::normalize_angle(yawE_-yawS_);   //约束两点角度误差  此处是错误的相对角度误差
    
    //存储目标
    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw = yawE_;
    //记录状态
    bool done = false;
    //d_t 100ci计算
    double scale = 1.0;
    double dScale = 0.01;

    while(!done) //出来2种情况 1 分割路径100次 足迹代价大于0的时候出来 完成任务
    {
      if(scale < 0)
      {
        target_x = start_x;
        target_y = start_y;
        target_yaw = yawS_;
        ROS_WARN("The carrot planner could not find a valid plan for this goal");
        break;
      }
        target_x = start_x + scale * diff_x; 
        target_y = start_y + scale * diff_y;
        target_yaw = angles::normalize_angle(yawS_ + scale * diff_yaw);
        double footprint_cost = footprintCost(target_x, target_y, target_yaw); //从目标点 往出发点遍历。检查是否有满足的 有的话弹出
        if(footprint_cost >= 0)  //代价大于0 出来 
        {
            done = true;
            ROS_INFO(" jump out");
        }
        scale -=dScale; //100次计算 delta   
    }
      plan.push_back(start);            //
      geometry_msgs::PoseStamped new_goal = goal;
      tf2::Quaternion goal_quat;
      goal_quat.setRPY(0, 0, target_yaw);
      new_goal.pose.position.x = target_x;
      new_goal.pose.position.y = target_y;
      new_goal.pose.orientation.x = goal_quat.x();
      new_goal.pose.orientation.y = goal_quat.y();
      new_goal.pose.orientation.z = goal_quat.z();
      new_goal.pose.orientation.w = goal_quat.w();

    plan.push_back(new_goal);
    OldTime =NewTime;
    ROS_INFO("x: [%f]  y:[%f]  the end x is [%f], y is [%f] size is [%d]",start_x,start_y,target_x,target_y,plan.size());
    return (done);
    // return  false;
}
};