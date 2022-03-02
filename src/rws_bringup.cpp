#include "yumi_bringup/rws_wrapper.h"
#include <ros/console.h>

int main(int argc, char **argv)
{
    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    // {
    //     ros::console::notifyLoggerLevelsChanged();
    // }
    ros::init(argc, argv, "yumi_bringup");
    ros::NodeHandle nh;

    RWSWrapper rwswrapper_l(RWSConstants::T_ROB_L);
    RWSWrapper rwswrapper_r(RWSConstants::T_ROB_R);

    ros::spin();
    return 0;
}