#pragma once

#include "ros/ros.h"



namespace RWSConstants
{

    struct RobTask
    {
        RobTask(std::string name, int topic_id, std::vector<std::string> joint_order,
                std::string buffer_filename, std::string as_execute_rapid, std::string as_followjoint):
        name(name), topic_id(topic_id),
        joint_order(joint_order), buffer_filename(buffer_filename), 
        as_execute_rapid(as_execute_rapid), as_followjoint(as_followjoint)
        {}
        ~RobTask()
        {}

        //name of the robot task in RWS protocol
        std::string name;
        // array index of the task in rws/sm state topic
        int topic_id;
        //whether our task state machine (sm) is currently running sth
        bool is_running = false;

        std::vector<std::string> joint_order;

        std::string buffer_filename;
        std::string as_execute_rapid;
        std::string as_followjoint;
    };

    // order sequence of joints to write to ABB controller for joint trajectory execution
    const std::vector<std::string> jointTrj_outputOrderL = {"yumi_robl_joint_1","yumi_robl_joint_2","yumi_robl_joint_3",
                                                            "yumi_robl_joint_4", "yumi_robl_joint_5", "yumi_robl_joint_6", "yumi_robl_joint_7"};
    

    const std::vector<std::string> jointTrj_outputOrderR = {"yumi_robr_joint_1","yumi_robr_joint_2","yumi_robr_joint_3",
                                                    "yumi_robr_joint_4", "yumi_robr_joint_5", "yumi_robr_joint_6", "yumi_robr_joint_7"};


    const double DEG_TO_RAD{M_PI/180.0};
    const double RAD_TO_DEG{180.0/M_PI};

    //Service Result Codes
    enum {
        RC_SUCCESS = 1u,
        RC_FAILED = 2u,
        RC_SERVER_IS_BUSY = 1001u,
        RC_EMPTY_FILENAME = 2001u,
        RC_EMPTY_SIGNAL_NAME = 2002u,
        RC_EMPTY_RAPID_TASK_NAME = 2003u,
        RC_EMPTY_RAPID_MODULE_NAME = 2004u,
        RC_EMPTY_RAPID_SYMBOL_NAME = 2005u,
        RC_NOT_IN_AUTO_MODE = 3001u,
        RC_MOTORS_ARE_OFF = 3002u,
        RC_MOTORS_ARE_ON = 3003u,
        RC_RAPID_NOT_STOPPED = 3004u,
        RC_RAPID_NOT_RUNNING = 3005u,
        RC_SM_RUNTIME_STATES_MISSING = 4001u,
        RC_SM_UNKNOWN_RAPID_TASK = 4002u,
        RC_SM_UNINITIALIZED = 4003u,
    };

    enum {
        SM_STATE_UNKNOWN = 1u,
        SM_STATE_IDLE = 2u,
        SM_STATE_INITIALIZE = 3u,
        SM_STATE_RUN_RAPID_ROUTINE = 4u,
        SM_STATE_RUN_EGM_ROUTINE = 5u,
        EGM_ACTION_UNKNOWN = 1u,
        EGM_ACTION_NONE = 2u,
        EGM_ACTION_RUN_JOINT = 3u,
        EGM_ACTION_RUN_POSE = 4u,
        EGM_ACTION_STOP = 5u,
        EGM_ACTION_START_STREAM = 6u,
        EGM_ACTION_STOP_STREAM = 7u,
    };

    const std::string BUFFER_FILE_L = "buffer_l.txt",
                        BUFFER_FILE_R = "buffer_r.txt";
    
    //JointTarget keyword for RAPID variable identification
    const std::string KW_JOINT_TARGETS = "jointtargets";

    namespace Routines
    {
        const std::string UPDATE_TRAJECTORY = "update_traj",
                            MOVE_JOINT = "movejoint";

    }

    namespace Services
    {
        const std::string EXECUTE_RAPID_NONBLOCKING = "execute_rapid_routine",
                            SET_RAPID_ROUTINE = "rws/sm_addin/set_rapid_routine",
                            RUN_RAPID_ROUTINE = "rws/sm_addin/run_rapid_routine",
                            SET_RAPID_SYMBOL = "rws/set_rapid_symbol",
                            SET_FILE_CONTENT = "rws/set_file_contents";

    }

    namespace Topics
    {
        const std::string SM_RUNTIME_STATES = "rws/sm_addin/runtime_states";
                            
    }

    namespace Actions
    {
        const std::string AS_EXECUTE_RAPID_L = "execute_rapid_l",
                            AS_EXECUTE_RAPID_R = "execute_rapid_r",
                            AS_FOLLOWJOINT_L = "joint_controller_l",
                            AS_FOLLOWJOINT_R = "joint_controller_r";
    }

    //Refernce Robot Task Instances for RAPID Execution
    const RobTask T_ROB_R("T_ROB_R", 0, jointTrj_outputOrderR, BUFFER_FILE_R, Actions::AS_EXECUTE_RAPID_R, Actions::AS_FOLLOWJOINT_R);
    const RobTask T_ROB_L("T_ROB_L", 1, jointTrj_outputOrderL, BUFFER_FILE_L, Actions::AS_EXECUTE_RAPID_L, Actions::AS_FOLLOWJOINT_L);
}