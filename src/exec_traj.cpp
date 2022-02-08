#include "yumi_bringup/trajectory_executer.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "exec_traj");
    ros::NodeHandle nh;

    TrajectoryExecuter executer("dadayeh");
    executer.temp_run();

    ros::spin();
    return 0;
}