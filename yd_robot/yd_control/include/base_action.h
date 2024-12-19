#ifndef _BASE_ACTION_H_
#define _BASE_ACTION_H_
#include <base_control.h>
#include <header_file.h>
#include <yd_control/BaseActionConfig.h>
using namespace std;
namespace basectl
{
//logic sub charege state  check simple goal state / check charge state is wait or not 
//first we check the staus is ok or not  check the pose2d is ok or not 

class Robot_LocalNav
{
    public:
            Robot_LocalNav(std::string client_name_,std::string server_name_);
            ~Robot_LocalNav(){}
            bool executeCycleCb(void){}
    private:
            RmClientNode RmClien_;                      //定义一个机械臂服务
            MoveLocal    BaseMoveLocalServer_;          //定一个机器人服务
            void ArmThread(void);

            void InitRobotLocalNav(void);
            std::string  armtask_type="charge";
            //is open action default is close false
            bool  action_open;
            //is repeat test  default is false
            bool repeatTest;
            //is setup the config
            bool configsetup_; 
            boost::thread* arm_thread_;
            //机械臂递归互斥锁
            boost::recursive_mutex    plan_mutex_;
            //机械臂条件变量 
            boost::condition_variable_any plan_condition_variable_;


            //动态调参互斥锁
            boost::recursive_mutex configure_mutex_;
            //动态服务接口  定义一个server drs_ydbase
            dynamic_reconfigure::Server<Base_Action::BaseActionConfig>  *drs_ydbase;
            //动态更新
            void reconfigureCallBack(Base_Action::BaseActionConfig &config_,uint32_t level_);
            Base_Action::BaseActionConfig last_action_config_;
            Base_Action::BaseActionConfig default_action_config_;

            ros::Subscriber ChargeStateSub;

};


}


#endif