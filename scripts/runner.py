#!/usr/bin/env python3

import time

import rospy

from abb_rapid_msgs.msg import *
from abb_robot_msgs.srv import *
from abb_robot_msgs.msg import *
from abb_rapid_sm_addin_msgs.srv import *
from abb_rapid_sm_addin_msgs.msg import RuntimeState

class RapidRunner():

    def __init__(self):
        self.is_running = False
        self.sub_sm_state = rospy.Subscriber("/yumi/rws/sm_addin/runtime_states",RuntimeState, self.cb_check_sm_state)
        
        self.rate = rospy.Rate(50)

    def set_run_rapid_routine(self, l_routine = None, r_routine = None):
        """
        Function: set_run_rapid_routine, to set and run RAPID Routine.
        """
        rospy.wait_for_service('/yumi/rws/sm_addin/set_rapid_routine')
        rospy.wait_for_service('/yumi/rws/sm_addin/run_rapid_routine')
        #
        set_rapid_routine = rospy.ServiceProxy('/yumi/rws/sm_addin/set_rapid_routine', SetRAPIDRoutine)
        run_rapid_routine = rospy.ServiceProxy('/yumi/rws/sm_addin/run_rapid_routine', TriggerWithResultCode)
        #
        if l_routine is not None:
            set_rapid_routine(task="T_ROB_L", routine=l_routine)
        if r_routine is not None:
            set_rapid_routine(task="T_ROB_R", routine=r_routine)
        rospy.sleep(0.5)
        #
        if (l_routine or r_routine) is not None:
            run_rapid_routine()
        rospy.sleep(0.5)
        while (runner.is_running and not rospy.is_shutdown()):
            self.rate.sleep()

    def cb_check_sm_state(self, msg: RuntimeState):
        self.is_running = any([sm.sm_state !=2 for sm in msg.state_machines])



if __name__ == '__main__':

    rospy.init_node('rapid_runner_node')

    test_list_l = ["dadayeh0", "dadayeh1"]

    runner = RapidRunner()
    
    r = rospy.Rate(15)

    for routine in test_list_l:
        runner.set_run_rapid_routine(l_routine=routine)


    