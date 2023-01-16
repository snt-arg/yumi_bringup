#!/usr/bin/env python3

import time

import rospy
import actionlib

from controller_manager_msgs.srv import SwitchController

from abb_rapid_msgs.msg import *
from abb_robot_msgs.srv import *
from abb_robot_msgs.msg import *
from abb_rapid_sm_addin_msgs.srv import *
from abb_rapid_sm_addin_msgs.msg import RuntimeState

from lfd_interface.msg import LFDPipelineAction, LFDPipelineGoal

from configurator import RWSRoutine, EGMRoutine, config

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
        while (self.is_running and not rospy.is_shutdown()):
            self.rate.sleep()

    def cb_check_sm_state(self, msg: RuntimeState):
        self.is_running = any([sm.sm_state !=2 for sm in msg.state_machines])


class EGMRunner():

    def __init__(self):
        self.client = actionlib.SimpleActionClient('lfd_pipeline', LFDPipelineAction)
        self.client.wait_for_server()

    def activate_egm(self):

        rospy.wait_for_service("/yumi/rws/sm_addin/start_egm_joint")
        rospy.wait_for_service("/yumi/egm/controller_manager/switch_controller")

        s_start_egm_joint = rospy.ServiceProxy("/yumi/rws/sm_addin/start_egm_joint", TriggerWithResultCode)
        s_switch_controller = rospy.ServiceProxy("/yumi/egm/controller_manager/switch_controller", SwitchController)

        s_start_egm_joint()
        rospy.sleep(0.5)
        s_switch_controller(start_controllers=["joint_trajectory_controller"], stop_controllers=[""], strictness=1, start_asap=False, timeout=0)

    def deactivate_egm(self):

        rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")
        s_stop_egm_joint = rospy.ServiceProxy("/yumi/rws/sm_addin/stop_egm", TriggerWithResultCode)
        s_stop_egm_joint()


    def run_egm(self, routine, duration):
        self.activate_egm()
        goal = LFDPipelineGoal(name=routine, duration=duration, train=True, execute=True)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        self.deactivate_egm()


if __name__ == '__main__':

    rospy.init_node('rapid_runner_node')

    rapid_runner = RapidRunner()
    egm_runner = EGMRunner()
    

    for record in config:
        if type(record) is RWSRoutine:
            rapid_runner.set_run_rapid_routine(record.l_routine, record.r_routine)
        elif type(record) is EGMRoutine:
            egm_runner.run_egm(record.name, record.duration)
