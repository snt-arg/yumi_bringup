#include "yumi_bringup/trajectory_executer.h"

TrajectoryExecuter::TrajectoryExecuter(RWSConstants::RobTask robtask):
ac_exec_rapid_(robtask.as_execute_rapid, true),
as_followjoint_(nh_, robtask.as_followjoint, boost::bind(&TrajectoryExecuter::executeCBFollowJoint, this, _1), false),
robtask_(robtask)
{
    ac_exec_rapid_.waitForServer();
    sc_setsymbol_ = nh_.serviceClient<abb_robot_msgs::SetRAPIDSymbol>(RWSConstants::Services::SET_RAPID_SYMBOL);
    sc_runrapid_ = nh_.serviceClient<abb_rapid_sm_addin_msgs::SetRAPIDRoutine>(RWSConstants::Services::EXECUTE_RAPID_NONBLOCKING);
    sc_writefile_ = nh_.serviceClient<abb_robot_msgs::SetFileContents>(RWSConstants::Services::SET_FILE_CONTENT);
    as_followjoint_.start();
}

TrajectoryExecuter::~TrajectoryExecuter()
{
}

void TrajectoryExecuter::executeCBFollowJoint(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    std::string trajectory = translateTrajectory(goal->trajectory);

    if (setFileContent(robtask_.buffer_filename, trajectory) &&
        runRoutine(RWSConstants::Routines::UPDATE_TRAJECTORY) &&
        runRoutine(RWSConstants::Routines::MOVE_JOINT))
    {
        //ROS_DEBUG_NAMED("RWS", "FollowJointTrajectory action executed successfully");
        result_followjoint_.error_code = result_followjoint_.SUCCESSFUL;
        as_followjoint_.setSucceeded(result_followjoint_);
    }
    else
    {
        result_followjoint_.error_code = result_followjoint_.INVALID_GOAL;
        as_followjoint_.setAborted(result_followjoint_);
    }

}

std::string TrajectoryExecuter::translateTrajectory(trajectory_msgs::JointTrajectory trajectory)
{
    std::string output;
    std::vector<int> indexmap;
    for (size_t i = 0; i < robtask_.joint_order.size(); i++)
    {
        indexmap.push_back(std::distance(trajectory.joint_names.begin(), 
                        std::find(trajectory.joint_names.begin(),trajectory.joint_names.end(), robtask_.joint_order[i]))
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


bool TrajectoryExecuter::runRoutine(std::string name)
{
    yumi_bringup::ExecuteRapidRoutineGoal goal;
    goal.routine = name;
    auto state = ac_exec_rapid_.sendGoalAndWait(goal);
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        return true;
    else
        ROS_DEBUG_NAMED("RWS", "Failed to run routine %s", name.c_str());
    return false;
}

bool TrajectoryExecuter::setFileContent(std::string filename, std::string content)
{
    abb_robot_msgs::SetFileContents srv;

    srv.request.filename=filename;
    srv.request.contents=content;
    if (sc_writefile_.call(srv) && srv.response.result_code == RWSConstants::RC_SUCCESS)
            return true;
    else
        ROS_DEBUG_NAMED("RWS", "Failed to write content on file %s, return code: %d, message: %s", 
                        srv.request.filename, srv.response.result_code, srv.response.message.c_str());
    return false;
}