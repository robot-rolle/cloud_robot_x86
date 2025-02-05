/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  enum MoveBaseState {
    PLANNING, // 正在规划路径的状态
    CONTROLLING, // 控制机器人运动的状态
    CLEARING // 规划路径或者控制失败，处于恢复或者清除
  };

  enum RecoveryTrigger
  {
    PLANNING_R,  // 全局规划失败
    CONTROLLING_R, // 局部轨迹规划失败
    OSCILLATION_R // 长时间在小区域运动
  };

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  // move_base是路径与速度规划的大类
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf2_ros::Buffer& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief  Performs a control cycle
       * @param goal A reference to the goal to pursue
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal);

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();
      void localPlanThread();

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf2_ros::Buffer& tf_;

      MoveBaseActionServer* as_;  // actionlib的服务器
      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;  // 局部路径规划器加载并创建实例后的指针
      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;   // 全局路径规划器
      boost::shared_ptr<nav_core::BaseGlobalPlanner> planerGRRT_;  


      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;  // 恢复行为，一般默认是转圈圈，也可以自定义其他的行为
      std::vector<std::string> recovery_behavior_names_;

      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;  //costmap 一个给规划 一个给控制

      std::string robot_base_frame_, global_frame_;

      unsigned int recovery_index_;

      geometry_msgs::PoseStamped global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_;
      uint32_t planning_retries_;
      double conservative_reset_dist_, clearing_radius_;  //
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      ros::Time last_stop_,last_start_; //上次停止 上次开始

      geometry_msgs::PoseStamped oscillation_pose_;

      
      // 以插件形式实现全局规划器、局部规划器和丢失时恢复规划器。
      // 插件形式可以实现随时动态地加载C++类库，但需要在包中注册该插件，不用这个的话需要提前链接（相当于运行时加载）
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      // pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> BaseGlobalPlugin_RRT_;
      // pluginlib::ClassLoader<nav_core::BaseLocalPlanner>  BaseGlobalPlugin_DynaticAstar_;

      // 保存规划器中刚刚算出的路径，然后传递planner_plan_ --> latest_plan_ --> controller_plan_
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      //作为一个桥梁，在MoveBase::executeCycle中传递给controller_plan_
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;




      // 全局规划线程相关的变量
      bool runPlanner_;
      // 递归锁
      // boost的一种结合了互斥锁的用法，可以使一个线程进入睡眠状态，然后在另一个线程触发唤醒。
      boost::recursive_mutex planner_mutex_; 
      // 通过这个值将goal在MoveBase::executeCb与MoveBase::planThread()之间传递
      // boost::condition_variable_any用来实现多线程同步，它必须与互斥量配合使用。
      boost::condition_variable_any planner_cond_;
      
      geometry_msgs::PoseStamped planner_goal_;

      boost::thread* planner_thread_;  //全局规划控制器


      boost::thread* local_planner_thread_; //局部规划
      boost::recursive_mutex configuration_mutex_;  //
      
      
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;
  };
};
#endif

