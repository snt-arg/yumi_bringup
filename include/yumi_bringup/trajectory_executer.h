#include "yumi_bringup/rws_wrapper.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <abb_robot_msgs/SetRAPIDSymbol.h>
#include <abb_robot_msgs/SetFileContents.h>

class TrajectoryExecuter
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_followjoint_;
    actionlib::SimpleActionClient<yumi_bringup::ExecuteRapidRoutineAction> ac_exec_rapid_;
    ros::ServiceClient sc_setsymbol_, sc_runrapid_, sc_writefile_;
    control_msgs::FollowJointTrajectoryResult result_followjoint_;

private:
    RWSConstants::RobTask robtask_;

public:
    TrajectoryExecuter(RWSConstants::RobTask robtask);
    ~TrajectoryExecuter();

private:
    bool setRapidSymbol(std::string symbol);
    std::string translateTrajectory(trajectory_msgs::JointTrajectory trajectory);
    bool runRoutine(std::string name);
    bool setFileContent(std::string filename, std::string content);
    void executeCBFollowJoint(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

};
