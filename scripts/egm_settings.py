#!/usr/bin/env python3

import rospy
import yaml
from abb_rapid_sm_addin_msgs.srv import SetEGMSettings, SetEGMSettingsRequest

def call_egm_service():

    rospy.init_node('egm_service_client')

    egm_config = rospy.get_param('~egm_config')

    rospy.wait_for_service('/yumi/rws/sm_addin/set_egm_settings')

    try:
        # Create a service proxy
        set_egm_settings = rospy.ServiceProxy('/yumi/rws/sm_addin/set_egm_settings', SetEGMSettings)

        # Read the request values from a YAML file
        with open(egm_config, 'r') as file:
            request_values = yaml.safe_load(file)
        print("Request values: ", request_values)
        # Create a request message with the provided data
        request = SetEGMSettingsRequest()
        request.task = request_values['task']
        request.settings.allow_egm_motions = request_values['settings']['allow_egm_motions']
        request.settings.use_presync = request_values['settings']['use_presync']
        request.settings.setup_uc.use_filtering = request_values['settings']['setup_uc']['use_filtering']
        request.settings.setup_uc.comm_timeout = request_values['settings']['setup_uc']['comm_timeout']
        request.settings.activate.tool.robhold = request_values['settings']['activate']['tool']['robhold']
        request.settings.activate.tool.tframe.trans.x = request_values['settings']['activate']['tool']['tframe']['trans']['x']
        request.settings.activate.tool.tframe.trans.y = request_values['settings']['activate']['tool']['tframe']['trans']['y']
        request.settings.activate.tool.tframe.trans.z = request_values['settings']['activate']['tool']['tframe']['trans']['z']
        request.settings.activate.tool.tframe.rot.q1 = request_values['settings']['activate']['tool']['tframe']['rot']['q1']
        request.settings.activate.tool.tframe.rot.q2 = request_values['settings']['activate']['tool']['tframe']['rot']['q2']
        request.settings.activate.tool.tframe.rot.q3 = request_values['settings']['activate']['tool']['tframe']['rot']['q3']
        request.settings.activate.tool.tframe.rot.q4 = request_values['settings']['activate']['tool']['tframe']['rot']['q4']
        request.settings.activate.tool.tload.mass = request_values['settings']['activate']['tool']['tload']['mass']
        request.settings.activate.tool.tload.cog.x = request_values['settings']['activate']['tool']['tload']['cog']['x']
        request.settings.activate.tool.tload.cog.y = request_values['settings']['activate']['tool']['tload']['cog']['y']
        request.settings.activate.tool.tload.cog.z = request_values['settings']['activate']['tool']['tload']['cog']['z']
        request.settings.activate.tool.tload.aom.q1 = request_values['settings']['activate']['tool']['tload']['aom']['q1']
        request.settings.activate.tool.tload.aom.q2 = request_values['settings']['activate']['tool']['tload']['aom']['q2']
        request.settings.activate.tool.tload.aom.q3 = request_values['settings']['activate']['tool']['tload']['aom']['q3']
        request.settings.activate.tool.tload.aom.q4 = request_values['settings']['activate']['tool']['tload']['aom']['q4']
        request.settings.activate.tool.tload.ix = request_values['settings']['activate']['tool']['tload']['ix']
        request.settings.activate.tool.tload.iy = request_values['settings']['activate']['tool']['tload']['iy']
        request.settings.activate.tool.tload.iz = request_values['settings']['activate']['tool']['tload']['iz']
        request.settings.activate.wobj.robhold = request_values['settings']['activate']['wobj']['robhold']
        request.settings.activate.wobj.ufprog = request_values['settings']['activate']['wobj']['ufprog']
        request.settings.activate.wobj.ufmec = request_values['settings']['activate']['wobj']['ufmec']
        request.settings.activate.wobj.uframe.trans.x = request_values['settings']['activate']['wobj']['uframe']['trans']['x']
        request.settings.activate.wobj.uframe.trans.y = request_values['settings']['activate']['wobj']['uframe']['trans']['y']
        request.settings.activate.wobj.uframe.trans.z = request_values['settings']['activate']['wobj']['uframe']['trans']['z']
        request.settings.activate.wobj.uframe.rot.q1 = request_values['settings']['activate']['wobj']['uframe']['rot']['q1']
        request.settings.activate.wobj.uframe.rot.q2 = request_values['settings']['activate']['wobj']['uframe']['rot']['q2']
        request.settings.activate.wobj.uframe.rot.q3 = request_values['settings']['activate']['wobj']['uframe']['rot']['q3']
        request.settings.activate.wobj.uframe.rot.q4 = request_values['settings']['activate']['wobj']['uframe']['rot']['q4']
        request.settings.activate.wobj.oframe.trans.x = request_values['settings']['activate']['wobj']['oframe']['trans']['x']
        request.settings.activate.wobj.oframe.trans.y = request_values['settings']['activate']['wobj']['oframe']['trans']['y']
        request.settings.activate.wobj.oframe.trans.z = request_values['settings']['activate']['wobj']['oframe']['trans']['z']
        request.settings.activate.wobj.oframe.rot.q1 = request_values['settings']['activate']['wobj']['oframe']['rot']['q1']
        request.settings.activate.wobj.oframe.rot.q2 = request_values['settings']['activate']['wobj']['oframe']['rot']['q2']
        request.settings.activate.wobj.oframe.rot.q3 = request_values['settings']['activate']['wobj']['oframe']['rot']['q3']
        request.settings.activate.wobj.oframe.rot.q4 = request_values['settings']['activate']['wobj']['oframe']['rot']['q4']
        request.settings.activate.correction_frame.trans.x = request_values['settings']['activate']['correction_frame']['trans']['x']
        request.settings.activate.correction_frame.trans.y = request_values['settings']['activate']['correction_frame']['trans']['y']
        request.settings.activate.correction_frame.trans.z = request_values['settings']['activate']['correction_frame']['trans']['z']
        request.settings.activate.correction_frame.rot.q1 = request_values['settings']['activate']['correction_frame']['rot']['q1']
        request.settings.activate.correction_frame.rot.q2 = request_values['settings']['activate']['correction_frame']['rot']['q2']
        request.settings.activate.correction_frame.rot.q3 = request_values['settings']['activate']['correction_frame']['rot']['q3']
        request.settings.activate.correction_frame.rot.q4 = request_values['settings']['activate']['correction_frame']['rot']['q4']
        request.settings.activate.sensor_frame.trans.x = request_values['settings']['activate']['sensor_frame']['trans']['x']
        request.settings.activate.sensor_frame.trans.y = request_values['settings']['activate']['sensor_frame']['trans']['y']
        request.settings.activate.sensor_frame.trans.z = request_values['settings']['activate']['sensor_frame']['trans']['z']
        request.settings.activate.sensor_frame.rot.q1 = request_values['settings']['activate']['sensor_frame']['rot']['q1']
        request.settings.activate.sensor_frame.rot.q2 = request_values['settings']['activate']['sensor_frame']['rot']['q2']
        request.settings.activate.sensor_frame.rot.q3 = request_values['settings']['activate']['sensor_frame']['rot']['q3']
        request.settings.activate.sensor_frame.rot.q4 = request_values['settings']['activate']['sensor_frame']['rot']['q4']
        request.settings.activate.cond_min_max = request_values['settings']['activate']['cond_min_max']
        request.settings.activate.lp_filter = request_values['settings']['activate']['lp_filter']
        request.settings.activate.sample_rate = request_values['settings']['activate']['sample_rate']
        request.settings.activate.max_speed_deviation = request_values['settings']['activate']['max_speed_deviation']
        request.settings.run.cond_time = request_values['settings']['run']['cond_time']
        request.settings.run.ramp_in_time = request_values['settings']['run']['ramp_in_time']
        request.settings.run.offset.trans.x = request_values['settings']['run']['offset']['trans']['x']
        request.settings.run.offset.trans.y = request_values['settings']['run']['offset']['trans']['y']
        request.settings.run.offset.trans.z = request_values['settings']['run']['offset']['trans']['z']
        request.settings.run.offset.rot.q1 = request_values['settings']['run']['offset']['rot']['q1']
        request.settings.run.offset.rot.q2 = request_values['settings']['run']['offset']['rot']['q2']
        request.settings.run.offset.rot.q3 = request_values['settings']['run']['offset']['rot']['q3']
        request.settings.run.offset.rot.q4 = request_values['settings']['run']['offset']['rot']['q4']
        request.settings.run.pos_corr_gain = request_values['settings']['run']['pos_corr_gain']
        request.settings.stop.ramp_out_time = request_values['settings']['stop']['ramp_out_time']
        # Call the service
        response = set_egm_settings(request)
        # Process the response if needed
        # For example, you can print the response
        rospy.loginfo("Service response: %s", response)

        # Call the service
        response = set_egm_settings(request)

        # Process the response if needed
        # For example, you can print the response
        rospy.loginfo("Service response: %s", response)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))

if __name__ == '__main__':
    call_egm_service()








































# import rospy
# from abb_rapid_sm_addin_msgs.srv import SetEGMSettings, SetEGMSettingsRequest # Replace with the actual service type

# def call_egm_service():
#     # Initialize the ROS node
#     rospy.init_node('egm_service_client')

#     # Wait for the service to become available
#     rospy.wait_for_service('/yumi/rws/sm_addin/set_egm_settings')

#     try:
#         # Create a service proxy
#         set_egm_settings = rospy.ServiceProxy('/yumi/rws/sm_addin/set_egm_settings', SetEGMSettings)

#         # Create a request message with the provided data
#         request = SetEGMSettingsRequest()
#         request.task = 'T_ROB_R'
#         request.settings.allow_egm_motions = True
#         request.settings.use_presync = False
#         request.settings.setup_uc.use_filtering = True
#         request.settings.setup_uc.comm_timeout = 1.0
#         request.settings.activate.tool.robhold = True
#         request.settings.activate.tool.tframe.trans.x = 0.0
#         request.settings.activate.tool.tframe.trans.y = 0.0
#         request.settings.activate.tool.tframe.trans.z = 0.0
#         request.settings.activate.tool.tframe.rot.q1 = 1.0
#         request.settings.activate.tool.tframe.rot.q2 = 0.0
#         request.settings.activate.tool.tframe.rot.q3 = 0.0
#         request.settings.activate.tool.tframe.rot.q4 = 0.0
#         request.settings.activate.tool.tload.mass = 0.0010000000474974513
#         request.settings.activate.tool.tload.cog.x = 0.0
#         request.settings.activate.tool.tload.cog.y = 0.0
#         request.settings.activate.tool.tload.cog.z = 0.0010000000474974513
#         request.settings.activate.tool.tload.aom.q1 = 1.0
#         request.settings.activate.tool.tload.aom.q2 = 0.0
#         request.settings.activate.tool.tload.aom.q3 = 0.0
#         request.settings.activate.tool.tload.aom.q4 = 0.0
#         request.settings.activate.tool.tload.ix = 0.0
#         request.settings.activate.tool.tload.iy = 0.0
#         request.settings.activate.tool.tload.iz = 0.0
#         request.settings.activate.wobj.robhold = False
#         request.settings.activate.wobj.ufprog = True
#         request.settings.activate.wobj.ufmec = ''
#         request.settings.activate.wobj.uframe.trans.x = 0.0
#         request.settings.activate.wobj.uframe.trans.y = 0.0
#         request.settings.activate.wobj.uframe.trans.z = 0.0
#         request.settings.activate.wobj.uframe.rot.q1 = 1.0
#         request.settings.activate.wobj.uframe.rot.q2 = 0.0
#         request.settings.activate.wobj.uframe.rot.q3 = 0.0
#         request.settings.activate.wobj.uframe.rot.q4 = 0.0
#         request.settings.activate.wobj.oframe.trans.x = 0.0
#         request.settings.activate.wobj.oframe.trans.y = 0.0
#         request.settings.activate.wobj.oframe.trans.z = 0.0
#         request.settings.activate.wobj.oframe.rot.q1 = 1.0
#         request.settings.activate.wobj.oframe.rot.q2 = 0.0
#         request.settings.activate.wobj.oframe.rot.q3 = 0.0
#         request.settings.activate.wobj.oframe.rot.q4 = 0.0
#         request.settings.activate.correction_frame.trans.x = 0.0
#         request.settings.activate.correction_frame.trans.y = 0.0
#         request.settings.activate.correction_frame.trans.z = 0.0
#         request.settings.activate.correction_frame.rot.q1 = 1.0
#         request.settings.activate.correction_frame.rot.q2 = 0.0
#         request.settings.activate.correction_frame.rot.q3 = 0.0
#         request.settings.activate.correction_frame.rot.q4 = 0.0
#         request.settings.activate.sensor_frame.trans.x = 0.0
#         request.settings.activate.sensor_frame.trans.y = 0.0
#         request.settings.activate.sensor_frame.trans.z = 0.0
#         request.settings.activate.sensor_frame.rot.q1 = 1.0
#         request.settings.activate.sensor_frame.rot.q2 = 0.0
#         request.settings.activate.sensor_frame.rot.q3 = 0.0
#         request.settings.activate.sensor_frame.rot.q4 = 0.0
#         request.settings.activate.cond_min_max = 0.5
#         request.settings.activate.lp_filter = 20.0
#         request.settings.activate.sample_rate = 4
#         request.settings.activate.max_speed_deviation = 100
#         request.settings.run.cond_time = 60.0
#         request.settings.run.ramp_in_time = 1.0
#         request.settings.run.offset.trans.x = 0.0
#         request.settings.run.offset.trans.y = 0.0
#         request.settings.run.offset.trans.z = 0.0
#         request.settings.run.offset.rot.q1 = 1.0
#         request.settings.run.offset.rot.q2 = 0.0
#         request.settings.run.offset.rot.q3 = 0.0
#         request.settings.run.offset.rot.q4 = 0.0
#         request.settings.run.pos_corr_gain = 1.0
#         request.settings.stop.ramp_out_time = 1.0

#         # Call the service
#         response = set_egm_settings(request)

#         # Process the response if needed
#         # For example, you can print the response
#         rospy.loginfo("Service response: %s", response)

#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s", str(e))

# if __name__ == '__main__':
#     call_egm_service()
