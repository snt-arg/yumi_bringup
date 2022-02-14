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
    actionlib::SimpleActionServer<yumi_bringup::ExecuteRapidRoutineAction> as_exec_rapid_;
    yumi_bringup::ExecuteRapidRoutineResult result_exec_rapid_;

    ros::Subscriber sub_rwsstate_;
    ros::ServiceServer ss_rapid_exec_;
    ros::ServiceClient sc_set_rapid_, sc_run_rapid_;

private:
    RWSConstants::RobTask robtask_;
    
public:
    RWSWrapper(RWSConstants::RobTask task);
    ~RWSWrapper();


private:
    bool serviceCBExecuteRapid(abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Request &req,
                                abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Response &res);
    
    bool executeRapid(std::string routine);
    
    void goalCBExecuteRapid();
    void subCBRapidState(const abb_rapid_sm_addin_msgs::RuntimeState::ConstPtr& msg);
};


