#include "yumi_bringup/trajectory_executer.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "exec_traj");
    ros::NodeHandle nh;

    TrajectoryExecuter executer(RWSConstants::T_ROB_L);
    //executer.temp_run();

    ros::spin();
    return 0;
}