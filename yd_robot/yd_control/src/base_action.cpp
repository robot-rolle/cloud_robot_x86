#include <base_action.h>

using namespace std;
namespace basectl
{
     Robot_LocalNav::Robot_LocalNav(std::string client_name,std::string server_name):
        RmClien_(client_name),
        BaseMoveLocalServer_(server_name),
        configsetup_(false),
        repeatTest(false)
     {
        InitRobotLocalNav();
    }

void Robot_LocalNav::InitRobotLocalNav(void)
    {
        ros::NodeHandle BaseAcNode("base_arm_action");//这个句柄订阅

        ros::NodeHandle SimpleBanode("simplenode");//这个句柄发布和configure 
        //初始化服务和响应接口
        drs_ydbase= new dynamic_reconfigure::Server<Base_Action::BaseActionConfig>(ros::NodeHandle("~"));
        dynamic_reconfigure::Server<Base_Action::BaseActionConfig>::CallbackType\
        cb=boost::bind(&Robot_LocalNav::reconfigureCallBack,this,_1,_2);
        drs_ydbase->setCallback(cb);
    }
/**
** @name:              reconfigureCallBack
** @brief:             rqt_configure回调函数
** @param:             
** @date:              2024-12-03
** @version:           V0.0
---------------------------------------*/
void Robot_LocalNav::reconfigureCallBack(Base_Action::BaseActionConfig &config_,uint32_t level_)
    {
        boost::recursive_mutex::scoped_lock configure_lock(configure_mutex_); //加锁 工作空间结束自动释放
        if(!configsetup_)
        {
            last_action_config_ =config_;
            default_action_config_ = config_;
            configsetup_ = true;
            return ;
        }
        if(config_.restore_defaults_) //check 是否要重置 reconfigue
        {
          config_= default_action_config_;   
          config_.restore_defaults_ =false; //防止干扰到其他变量
        }
        // config_ param set to  obeject
        repeatTest = config_.repeatTest_;
        action_open = config_.action_open_;
        armtask_type = config_.armtask_type_;
        last_action_config_ = config_;
    }
/**
** @name:              ArmThread
** @brief:             arm thread task
** @param:             none
** @date:              2024-12-04
** @version:           V0.0
---------------------------------------*/
void Robot_LocalNav::ArmThread(void)
    {
       ros::NodeHandle arm_n;
       ros::Timer task_timer;
       bool wait_for_wake_ = false;
       boost::unique_lock<boost::recursive_mutex> lock(plan_mutex_);
       while (arm_n.ok())
       {
        while (wait_for_wake_ )
        {
            /* code */
        }
        
       }
       
       
    }
}