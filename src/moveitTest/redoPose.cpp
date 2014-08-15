#include<ros/ros.h>
#include<actionlib/client/simple_action_client.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<jaco_msgs/ArmJointAnglesAction.h>
#include<control_msgs/FollowJointTrajectoryAction.h>
#include <angles/angles.h>
#include <jaco_driver/jaco_comm.h>
#include <boost/thread/recursive_mutex.hpp>

#include <fstream>

void parseTextFile(std::string p_path, std::vector<jaco_msgs::ArmJointAnglesGoal>& p_vectorGoal)
{
    std::ifstream ifs;
    ifs.open(p_path.c_str());
    if(ifs.is_open())
    {
        std::string line;
        int nbLine = 0;
        jaco_msgs::ArmJointAnglesGoal goalTemp;
        std::vector<double> position;
        while(std::getline(ifs, line))
        {
            if(nbLine == 6)
            {
                nbLine = 0;
                goalTemp.angles.joint1 = angles::to_degrees(position[0]);
                goalTemp.angles.joint2 = angles::to_degrees(position[1]);
                goalTemp.angles.joint3 = angles::to_degrees(position[2]);
                goalTemp.angles.joint4 = angles::to_degrees(position[3]);
                goalTemp.angles.joint5 = angles::to_degrees(position[4]);
                goalTemp.angles.joint6 = angles::to_degrees(position[5]);
                position.clear();
                p_vectorGoal.push_back(goalTemp);
            }
            else
            {
                position.push_back(atof(line.c_str()));
                nbLine++;
            }
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "redo_pose_action");

    ros::NodeHandle nParam("~");
    ros::NodeHandle nh;

    std::string logFile;
    nParam.param("log_file", logFile, std::string(""));

    actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/jaco_arm_driver/joint_angles/joint_angles", true);

    //std::vector<trajectory_msgs::JointTrajectoryPoint> pointVector(p_input->trajectory.points);

    std::vector<jaco_msgs::ArmJointAnglesGoal> pointVector;
    parseTextFile(logFile, pointVector);
    std::cout << "File" << logFile << std::endl;

    std::cout << "Point size" << pointVector.size() << std::endl;

    for(int i = 0; i < pointVector.size(); i++)
    {
/*
        ac.sendGoal(pointVector.at(i));
        //debug
        std::cout << ros::Time::now() << std::endl;
        std::cout << "Position1 == " << pointVector.at(i).angles.joint1 << std::endl;
        std::cout << "Position2 == " << pointVector.at(i).angles.joint2 << std::endl;
        std::cout << "Position3 == " << pointVector.at(i).angles.joint3 << std::endl;
        std::cout << "Position4 == " << pointVector.at(i).angles.joint4 << std::endl;
        std::cout << "Position5 == " << pointVector.at(i).angles.joint5 << std::endl;
        std::cout << "Position6 == " << pointVector.at(i).angles.joint6 << std::endl;
        std::cout << "===============================" << std::endl;
        /////////////////////////////////////////////////////////////

        ac.waitForResult(ros::Duration(1.0));
*/
    }/*
        boost::recursive_mutex mx;
        jaco::JacoComm com(nParam, mx, false);
        jaco::JacoAngles angles;
        angles.Actuator1 = 7;
        angles.Actuator2 = -1.73;
        angles.Actuator3 = 0.701;
        angles.Actuator4 = -0.81;
        angles.Actuator5 = 1.51;
        angles.Actuator6 = 3.136;
        com.setJointAngles(angles, 1);
        */

    //control_msgs::FollowJointTrajectoryResult result;
    //p_server->setSucceeded(result);

    return 0;
}
