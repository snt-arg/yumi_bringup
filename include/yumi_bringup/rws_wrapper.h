#include "ros/ros.h"

#include <abb_rapid_sm_addin_msgs/SetRAPIDRoutine.h>
#include <abb_rapid_sm_addin_msgs/RuntimeState.h>
#include <abb_robot_msgs/TriggerWithResultCode.h>

#include <actionlib/server/simple_action_server.h>
#include "yumi_bringup/ExecuteRapidRoutineAction.h"

class RobTask
{
public:
    RobTask(std::string name, int topic_id);
    ~RobTask();

public:
    std::string name;
    int topic_id;
    bool is_running = false;
};

RobTask::RobTask(std::string name, int topic_id):
name(name), topic_id(topic_id)
{}

class RWSWrapper
{

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<yumi_bringup::ExecuteRapidRoutineAction> as_rapid_exec_;
    yumi_bringup::ExecuteRapidRoutineResult result_;

    ros::Subscriber sub_rwsstate_;

private:
    enum YumiTask { T_ROB_R = 0, T_ROB_L};
    ros::ServiceServer service;
    ros::ServiceClient setter_client, runner_client;
    std::map<std::string, bool> task_status = {{"T_ROB_R" , false},
                                                {"T_ROB_L", false}};
    int running_task_;
    std::string running_routine_;

    RobTask task;
    

public:
    RWSWrapper(RobTask task);
    ~RWSWrapper();
    void open();


private:
    bool cb_execute_rapid_routine(abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Request &req,
                                abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Response &res);
    
    bool execute_rapid_routine(std::string task, std::string routine);
    
    void cb_rapid_exec_goal();
    void cb_rapid_exec_analysis(const abb_rapid_sm_addin_msgs::RuntimeState::ConstPtr& msg);


};



RWSWrapper::~RWSWrapper()
{
}
