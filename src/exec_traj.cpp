#include "yumi_bringup/trajectory_executer.h"
#include <boost/thread.hpp>

void spinThread_L()
{
    TrajectoryExecuter executer(RWSConstants::T_ROB_L);
    ros::waitForShutdown();
}

void spinThread_R()
{
    TrajectoryExecuter executer(RWSConstants::T_ROB_R);
    ros::waitForShutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exec_traj");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    boost::thread thread_l(&spinThread_L);
    boost::thread thread_r(&spinThread_R);

    thread_l.join();
    thread_r.join();
    return 0;
}