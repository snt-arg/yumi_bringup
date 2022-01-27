#include "yumi_bringup/rws_wrapper.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "yumi_bringup");
    ros::NodeHandle nh;

    RWSWrapper rwswrapper;
    rwswrapper.open();

    ros::spin();
    return 0;
}