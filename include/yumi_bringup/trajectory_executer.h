#include "yumi_bringup/rws_wrapper.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <abb_robot_msgs/SetRAPIDSymbol.h>
#include <abb_robot_msgs/SetFileContents.h>

#include <moveit_msgs/RobotTrajectory.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

class TrajectoryExecuter
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_followjoint;
    actionlib::SimpleActionClient<yumi_bringup::ExecuteRapidRoutineAction> ac_exec;

private:
    ros::ServiceClient client_setsymbol, client_runrapid, client_writefile;

    control_msgs::FollowJointTrajectoryResult result_followjoint;

public:
    TrajectoryExecuter(std::string ac_name);
    ~TrajectoryExecuter();
    void temp_run();

private:
    bool set_rapid_symbol(std::string symbol);
    std::string translate_trajectory(trajectory_msgs::JointTrajectory trajectory);
    bool run_routine(std::string name);
    bool set_file_content(std::string filename, std::string content);

    void cb_execute(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

    trajectory_msgs::JointTrajectory temp_create_trajectory();


    std::map<std::string, int> joint_mapping_l = {
        {"yumi_robl_joint_1", 0},
        {"yumi_robl_joint_2", 1},
        {"yumi_robl_joint_3", 2},
        {"yumi_robl_joint_4", 3},
        {"yumi_robl_joint_5", 4},
        {"yumi_robl_joint_6", 5},
        {"yumi_robl_joint_7", 6}
    };

};
