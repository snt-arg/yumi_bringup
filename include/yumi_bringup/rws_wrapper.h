#include "ros/ros.h"

#include <abb_rapid_sm_addin_msgs/SetRAPIDRoutine.h>
#include <abb_rapid_sm_addin_msgs/RuntimeState.h>
#include <abb_robot_msgs/TriggerWithResultCode.h>
#include <actionlib/server/simple_action_server.h>

#include "yumi_bringup/ExecuteRapidRoutineAction.h"

#include <yumi_bringup/utilities.h>

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
    RWSConstants::RobTask robtask;
    

public:
    RWSWrapper(RWSConstants::RobTask task);
    ~RWSWrapper();
    void open();


private:
    bool cb_execute_rapid_routine(abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Request &req,
                                abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Response &res);
    
    bool execute_rapid_routine(std::string routine);
    
    void cb_rapid_exec_goal();
    void cb_rapid_exec_analysis(const abb_rapid_sm_addin_msgs::RuntimeState::ConstPtr& msg);


};


