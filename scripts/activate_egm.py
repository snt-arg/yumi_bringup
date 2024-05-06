#!/usr/bin/env python3

import time

import rospy
import actionlib

from controller_manager_msgs.srv import SwitchController

from abb_rapid_msgs.msg import *
from abb_robot_msgs.srv import *
from abb_robot_msgs.msg import *
from abb_rapid_sm_addin_msgs.srv import *


def activate_egm():

    rospy.wait_for_service("/yumi/rws/sm_addin/start_egm_joint")
    rospy.wait_for_service("/yumi/egm/controller_manager/switch_controller")

    s_start_egm_joint = rospy.ServiceProxy("/yumi/rws/sm_addin/start_egm_joint", TriggerWithResultCode)
    s_switch_controller = rospy.ServiceProxy("/yumi/egm/controller_manager/switch_controller", SwitchController)

    s_start_egm_joint()
    rospy.sleep(0.5)
    s_switch_controller(start_controllers=["joint_trajectory_controller"], stop_controllers=[""], strictness=1, start_asap=False, timeout=0)



if __name__ == '__main__':

    rospy.init_node('activate_egm_node')
    activate_egm()