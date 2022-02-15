#include "yumi_bringup/rws_wrapper.h"


bool RWSWrapper::serviceCBExecuteRapid(abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Request &req,
                            abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Response &res) 
{
    if(executeRapid(req.routine))
        res.result_code = RWSConstants::RC_SUCCESS;
    else
        res.result_code = RWSConstants::RC_FAILED;
    
    return true;
}


bool RWSWrapper::executeRapid(std::string routine)
{
    abb_rapid_sm_addin_msgs::SetRAPIDRoutine set_srv;
    abb_robot_msgs::TriggerWithResultCode run_srv;

    set_srv.request.task = robtask_.name;
    set_srv.request.routine = routine;

    if(sc_set_rapid_.call(set_srv) && set_srv.response.result_code == RWSConstants::RC_SUCCESS)
    {
        if (sc_run_rapid_.call(run_srv) && run_srv.response.result_code == RWSConstants::RC_SUCCESS)
        {
            ROS_DEBUG_NAMED("RWS", "Rapid routine %s execution from %s started", set_srv.request.routine.c_str(), robtask_.name.c_str());
            return true;
        }
        else
            ROS_DEBUG_NAMED("RWS", "Failed to run rapid routine %s, return code: %d, message: %s", 
                            set_srv.request.routine.c_str(), run_srv.response.result_code, run_srv.response.message.c_str());  
    }
    else
        ROS_DEBUG_NAMED("RWS", "Failed to set rapid routine %s, return code: %d, message: %s",
                             set_srv.request.routine.c_str(), set_srv.response.result_code, set_srv.response.message.c_str());

    return false;    

}

void RWSWrapper::goalCBExecuteRapid()
{
    if (!robtask_.is_running)
    {
        //ros::Duration(0.5).sleep();
        auto goal = as_exec_rapid_.acceptNewGoal();
        if(executeRapid(goal->routine))
            robtask_.is_running = true;
        else
            ROS_DEBUG_NAMED("RWS", "Failed to execute rapid action");
    }
    else
        ROS_DEBUG_NAMED("RWS", "Recieved a rapid execution request while another one is running, dropping request");
}

void RWSWrapper::subCBRapidState(const abb_rapid_sm_addin_msgs::RuntimeState::ConstPtr& msg)
{
    if (!as_exec_rapid_.isActive())
        return;
    
    for (auto state : msg->state_machines)
    {
        if (state.rapid_task == robtask_.name && state.sm_state == RWSConstants::SM_STATE_IDLE)
        {
            robtask_.is_running = false;
            result_exec_rapid_.result_code = RWSConstants::RC_SUCCESS;
            as_exec_rapid_.setSucceeded(result_exec_rapid_);
            ROS_DEBUG_NAMED("RWS", "task %s is now IDLE", robtask_.name.c_str());
        }
    }
    // if(msg->state_machines[robtask_.topic_id].sm_state == RWSConstants::SM_STATE_IDLE)
    // {

    // }
}

RWSWrapper::RWSWrapper(RWSConstants::RobTask task) :
as_exec_rapid_(nh_, task.as_execute_rapid, false),
robtask_(task)
{
    //ss_rapid_exec_ = nh_.advertiseService(RWSConstants::Services::EXECUTE_RAPID_NONBLOCKING, &RWSWrapper::serviceCBExecuteRapid, this);
    sc_set_rapid_ = nh_.serviceClient<abb_rapid_sm_addin_msgs::SetRAPIDRoutine>(RWSConstants::Services::SET_RAPID_ROUTINE);
    sc_run_rapid_ = nh_.serviceClient<abb_robot_msgs::TriggerWithResultCode>(RWSConstants::Services::RUN_RAPID_ROUTINE);

    as_exec_rapid_.registerGoalCallback(boost::bind(&RWSWrapper::goalCBExecuteRapid, this));

    sub_rwsstate_ = nh_.subscribe(RWSConstants::Topics::SM_RUNTIME_STATES, 1,&RWSWrapper::subCBRapidState,this);

    as_exec_rapid_.start();
}

RWSWrapper::~RWSWrapper() {}