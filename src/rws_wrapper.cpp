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
    if(execute_rapid_routine(req.task, req.routine))
        res.result_code = 1;
    
    return true;
}


bool RWSWrapper::execute_rapid_routine(std::string task, std::string routine)
{
    ROS_INFO("task is %s  and status is %d", task.c_str(), task_status[task]);
    if (task_status[task])
        return false;

    abb_rapid_sm_addin_msgs::SetRAPIDRoutine set_srv;
    abb_robot_msgs::TriggerWithResultCode run_srv;

    set_srv.request.task = task;
    set_srv.request.routine = routine;


    //ROS_INFO(set_srv.request.task.c_str());
    if(setter_client.call(set_srv) && set_srv.response.result_code == 1)
    {
        //ROS_INFO("set rapid routine response is %d", set_srv.response.result_code);
        if (runner_client.call(run_srv) && run_srv.response.result_code == 1)
        {
            ROS_INFO("RAPID routine executed successfully");
            task_status[task] = true;
            return true;
        }
        
    }

    return false;    

}

void RWSWrapper::cb_rapid_exec_goal()
{
    auto goal = as_rapid_exec_.acceptNewGoal();
    if(!execute_rapid_routine(goal->task, goal->routine))
    {
        ROS_INFO("dadayehhhh");
        //result_.result_code = 2;
        //as_rapid_exec_.setAborted(result_);
    }

    
}

void RWSWrapper::cb_rapid_exec_analysis(const abb_rapid_sm_addin_msgs::RuntimeState::ConstPtr& msg)
{
    if (!as_rapid_exec_.isActive())
        return;
    
    if(task_status["T_ROB_R"] && msg->state_machines[T_ROB_R].sm_state == 2)
    {
        task_status["T_ROB_R"] = false;
        result_.result_code = 1;
        as_rapid_exec_.setSucceeded(result_);
    }
    if(task_status["T_ROB_L"] && msg->state_machines[T_ROB_L].sm_state == 2)
    {
        task_status["T_ROB_L"] = false;
        result_.result_code = 1;
        as_rapid_exec_.setSucceeded(result_);
    }
}

RWSWrapper::RWSWrapper(RobTask task) :
as_rapid_exec_(nh_,"dadayeh", false), task(task)
{
    
}