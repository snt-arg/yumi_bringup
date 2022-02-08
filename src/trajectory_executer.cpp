#include "yumi_bringup/trajectory_executer.h"

TrajectoryExecuter::TrajectoryExecuter(std::string ac_name):
as_(ac_name, true)
{
    as_.waitForServer();
    client_setsymbol = nh_.serviceClient<abb_robot_msgs::SetRAPIDSymbol>("/yumi/rws/set_rapid_symbol");
    client_runrapid = nh_.serviceClient<abb_rapid_sm_addin_msgs::SetRAPIDRoutine>("execute_rapid_routine");
    client_writefile = nh_.serviceClient<abb_robot_msgs::SetFileContents>("/yumi/rws/set_file_contents");
}

TrajectoryExecuter::~TrajectoryExecuter()
{
}


std::string TrajectoryExecuter::translate_trajectory()
{

    std::string traj;

    // std::string p0 = "1.7159965299579243,0.7417649320975901,-1.2807212255959943,1.0423085866464574,2.5031373328146658,1.203979309308162,-0.3586770323253938";
    // std::string p1 = "1.6993605615765264,0.6094677444159963,-1.4568820080117382,0.7954274949046322,2.1751549298176305,1.028475861628477,-0.1419745158716965";
    // std::string p2 = "1.7572586157812304,0.3285640040895953,-1.818982167733779,0.2554863644351166,1.4192904239608337,1.0301429338918393,0.5028663994071683";
    // std::string p3 = "2.4262188489465615,0.4246291847462837,-2.539293473038018,0.6683439776890865,1.4180118403214867,1.0606846644841132,0.3084473174529774";
    // std::string p4 = "2.169501837771719,0.5451138215442901,-2.1061588175602934,1.0429908219387085,1.8952495087153591,1.0868974933034907,-0.12011995849861214";

    std::string p0 = "98.3193588256836,42.5,-73.37992095947266,59.71988296508789,143.41920471191406,68.9829330444336,-20.55068016052246";
    std::string p1 = "97.3661880493164, 34.91992950439453, -83.47319030761719, 45.57463836669922, 124.627197265625, 58.92732620239258, -8.134540557861328";
    std::string p2 = "85.03775787353516, 29.18630027770996, -79.63162994384766, 20.48866081237793, 102.64969635009766, 52.858463287353516, 14.287779808044434";
    std::string p3 = "100.68350219726562, 18.82533073425293, -104.22000122070312, 14.638290405273438, 81.31935119628906, 59.02284240722656, 28.812122344970703";
    std::string p4 = "139.01210021972656, 24.32946014404297, -145.4907989501953, 38.29328918457031, 81.24609375, 60.77275466918945, 17.6727294921875";


    traj = "jointtargets,5," + p0 + "," + p1 + "," + p2 + "," + p3 + "," + p4;
    return traj;
}


bool TrajectoryExecuter::run_routine(std::string name)
{
    yumi_bringup::ExecuteRapidRoutineGoal goal;
    goal.routine = name;
    auto state = as_.sendGoalAndWait(goal);
    ROS_INFO("Action finished: %s",state.toString().c_str());
    return true;
}

bool TrajectoryExecuter::set_file_content(std::string filename, std::string content)
{
    abb_robot_msgs::SetFileContents srv;

    srv.request.filename=filename;
    srv.request.contents=content;
    if (client_writefile.call(srv))
        if(srv.response.result_code == 1)
            return true;
        else return false;
}



void TrajectoryExecuter::temp_run()
{
    std::string trajectory = translate_trajectory();

    set_file_content("buffer.txt", trajectory);
    
    ROS_INFO("Finished buffer");

}



