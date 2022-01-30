#include "yumi_bringup/rws_wrapper.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "yumi_bringup");
    ros::NodeHandle nh;

    RobTask T_ROB_L("T_ROB_L");

    RWSWrapper rwswrapper(T_ROB_L);
    rwswrapper.open();

    ros::spin();
    return 0;
}