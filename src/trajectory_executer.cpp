#include "yumi_bringup/trajectory_executer.h"

TrajectoryExecuter::TrajectoryExecuter(std::string ac_name):
ac_exec(ac_name, true), as_followjoint(nh_, RWSConstants::Actions::AS_FOLLOWJOINT_L, boost::bind(&TrajectoryExecuter::cb_execute, this, _1), false)
{
    ac_exec.waitForServer();
    as_followjoint.start();
    client_setsymbol = nh_.serviceClient<abb_robot_msgs::SetRAPIDSymbol>(RWSConstants::Services::SET_RAPID_SYMBOL);
    client_runrapid = nh_.serviceClient<abb_rapid_sm_addin_msgs::SetRAPIDRoutine>(RWSConstants::Services::EXECUTE_RAPID_NONBLOCKING);
    client_writefile = nh_.serviceClient<abb_robot_msgs::SetFileContents>(RWSConstants::Services::SET_FILE_CONTENT);
}

TrajectoryExecuter::~TrajectoryExecuter()
{
}

void TrajectoryExecuter::cb_execute(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    std::string trajectory = translate_trajectory(goal->trajectory);
    //std::string trajectory = translate_trajectory(temp_create_trajectory());

    set_file_content(RWSConstants::BUFFER_FILE_L, trajectory);
    run_routine(RWSConstants::Routines::UPDATE_TRAJECTORY);
    run_routine(RWSConstants::Routines::MOVE_JOINT);
    ROS_INFO("Finished action execution");
    result_followjoint.error_code = result_followjoint.SUCCESSFUL;
    as_followjoint.setSucceeded(result_followjoint);
}

std::string TrajectoryExecuter::translate_trajectory(trajectory_msgs::JointTrajectory trajectory)
{
    std::string output;
    std::vector<int> indexmap;
    for (size_t i = 0; i < RWSConstants::jointTrj_outputOrderL.size(); i++)
    {
        indexmap.push_back(std::distance(trajectory.joint_names.begin(), 
                        std::find(trajectory.joint_names.begin(),trajectory.joint_names.end(), RWSConstants::jointTrj_outputOrderL[i]))
                        );   
    }
    for (size_t i = 0; i < trajectory.points.size(); i++)
    {
        for (int idx : indexmap)
        {
            output += std::to_string(RWSConstants::RAD_TO_DEG * trajectory.points[i].positions[idx]);
            output += ",";
        }
    }
    
    output = RWSConstants::KW_JOINT_TARGETS + "," + std::to_string(trajectory.points.size()) + "," + output;
    return output;
}


bool TrajectoryExecuter::run_routine(std::string name)
{
    yumi_bringup::ExecuteRapidRoutineGoal goal;
    goal.routine = name;
    auto state = ac_exec.sendGoalAndWait(goal);
    ROS_INFO("Action finished: %s",state.toString().c_str());
    return true;
}

bool TrajectoryExecuter::set_file_content(std::string filename, std::string content)
{
    abb_robot_msgs::SetFileContents srv;

    srv.request.filename=filename;
    srv.request.contents=content;
    if (client_writefile.call(srv))
        if(srv.response.result_code == RWSConstants::RC_SUCCESS)
            return true;
        else return false;
}

void TrajectoryExecuter::temp_run()
{
    //std::string trajectory = translate_trajectory(temp_create_trajectory());

    //set_file_content("buffer_l.txt", trajectory);
    
    //ROS_INFO("Finished buffer");

}