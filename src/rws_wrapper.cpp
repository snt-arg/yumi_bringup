#include "yumi_bringup/rws_wrapper.h"

void RWSWrapper::open()
{
    service = nh_.advertiseService("execute_rapid_routine", &RWSWrapper::cb_execute_rapid_routine, this);
    setter_client = nh_.serviceClient<abb_rapid_sm_addin_msgs::SetRAPIDRoutine>("/yumi/rws/sm_addin/set_rapid_routine");
    runner_client = nh_.serviceClient<abb_robot_msgs::TriggerWithResultCode>("/yumi/rws/sm_addin/run_rapid_routine");

    as_rapid_exec_.registerGoalCallback(boost::bind(&RWSWrapper::cb_rapid_exec_goal, this));
    sub_rwsstate_ = nh_.subscribe("/yumi/rws/sm_addin/runtime_states", 1,&RWSWrapper::cb_rapid_exec_analysis,this);

    as_rapid_exec_.start();
}


bool RWSWrapper::cb_execute_rapid_routine(abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Request &req,
                            abb_rapid_sm_addin_msgs::SetRAPIDRoutine::Response &res) 
{
    if(execute_rapid_routine(req.routine))
        res.result_code = 1;
    
    return true;
}


bool RWSWrapper::execute_rapid_routine(std::string routine)
{
    abb_rapid_sm_addin_msgs::SetRAPIDRoutine set_srv;
    abb_robot_msgs::TriggerWithResultCode run_srv;

    set_srv.request.task = robtask.name;
    set_srv.request.routine = routine;


    //ROS_INFO(set_srv.request.task.c_str());
    if(setter_client.call(set_srv) && set_srv.response.result_code == 1)
    {
        //ROS_INFO("set rapid routine response is %d", set_srv.response.result_code);
        if (runner_client.call(run_srv) && run_srv.response.result_code == 1)
        {
            ROS_INFO("RAPID routine executed successfully");
            return true;
        }
        
    }

    return false;    

}

void RWSWrapper::cb_rapid_exec_goal()
{
    if (!robtask.is_running)
    {
        auto goal = as_rapid_exec_.acceptNewGoal();
        if(execute_rapid_routine(goal->routine))
            robtask.is_running = true;
        else
            ROS_INFO("ridi");
    }
}

void RWSWrapper::cb_rapid_exec_analysis(const abb_rapid_sm_addin_msgs::RuntimeState::ConstPtr& msg)
{
    if (!as_rapid_exec_.isActive())
        return;
    
    if(msg->state_machines[robtask.topic_id].sm_state == 2)
    {
        robtask.is_running = false;
        result_.result_code = 1;
        as_rapid_exec_.setSucceeded(result_);
    }
}

RWSWrapper::RWSWrapper(RobTask task) :
as_rapid_exec_(nh_,"dadayeh", false), robtask(task)
{
    
}

RobTask::RobTask(std::string name):
name(name)
{
    topic_id = task_id[name];
}

RobTask::~RobTask() {}

RWSWrapper::~RWSWrapper() {}