#include "ros/ros.h"

#include <abb_rapid_sm_addin_msgs/SetRAPIDRoutine.h>
#include <abb_rapid_sm_addin_msgs/RuntimeState.h>
#include <abb_robot_msgs/TriggerWithResultCode.h>

#include <actionlib/server/simple_action_server.h>
#include "yumi_bringup/ExecuteRapidRoutineAction.h"

class RobTask
{
public:
    RobTask(std::string name);
    ~RobTask();

public:
    std::string name;
    int topic_id;
    bool is_running = false;

private:
    std::map<std::string, int> task_id = {{"T_ROB_R" , 0},
                                        {"T_ROB_L", 1}};
};

class RWSWrapper
{

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<yumi_bringup::ExecuteRapidRoutineAction> as_rapid_exec_;
    yumi_bringup::ExecuteRapidRoutineResult result_;

    ros::Subscriber sub_rwsstate_;

private:
    ros::ServiceServer service;
    ros::ServiceClient setter_client, runner_client;
    RobTask robtask;
    

public:
    RWSWrapper(RobTask task);
    ~RWSWrapper();
    void open();


private:
    bool cb_execute_rapid_routine(abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Request &req,
                                abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Response &res);
    
    bool execute_rapid_routine(std::string routine);
    
    void cb_rapid_exec_goal();
    void cb_rapid_exec_analysis(const abb_rapid_sm_addin_msgs::RuntimeState::ConstPtr& msg);


};


