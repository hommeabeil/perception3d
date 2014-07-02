#include "jaco_custom.h"

using namespace std;

JacoCustom::JacoCustom(ros::NodeHandle &node){
    end_program = false;

    arm_is_stopped = false;
    check_arm_status = false;
    arm_is_stopped_counter = 0;

    fingers_are_stopped = false;
    check_fingers_status = false;
    fingers_are_stopped_counter = 0;
}


void JacoCustom::arm_position_callback (const geometry_msgs::PoseStampedConstPtr& input_pose){

    if(check_arm_status){
        bool same_pose = is_same_pose(&arm_pose,input_pose);
        if(same_pose){
            arm_is_stopped_counter++;
        }
        if(arm_is_stopped_counter >= 5){
            arm_is_stopped = true;
        }
    }

    // Assign new value to arm pose
    arm_mutex.lock();
    arm_pose = *input_pose;
    arm_mutex.unlock();

}


void JacoCustom::fingers_position_callback(const jaco_msgs::FingerPositionConstPtr& input_fingers){
    if(check_fingers_status){
        bool same_pose = is_same_pose(&fingers_pose,input_fingers); //create a new function
        if(same_pose){
            fingers_are_stopped_counter++;
        }
        if(fingers_are_stopped_counter >= 5){
            fingers_are_stopped = true;
        }
    }

    // Assign new value to arm pose
    fingers_mutex.lock();
    fingers_pose = *input_fingers;
    fingers_mutex.unlock();

    cout << "finger 1 : " << input_fingers->Finger_1 << endl;

}


void JacoCustom::open_fingers(){
    actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> action_client("jaco/finger_joint_angles",true);
    action_client.waitForServer();
    jaco_msgs::SetFingersPositionGoal fingers = jaco_msgs::SetFingersPositionGoal();
    fingers.fingers.Finger_1 = 0;
    fingers.fingers.Finger_2 = 0;
    fingers.fingers.Finger_3 = 0;
    action_client.sendGoal(fingers);
    wait_for_fingers_stopped();
}

void JacoCustom::close_fingers(){
    actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> action_client("jaco/finger_joint_angles",true);
    action_client.waitForServer();
    jaco_msgs::SetFingersPositionGoal fingers = jaco_msgs::SetFingersPositionGoal();
    fingers.fingers.Finger_1 = 60;
    fingers.fingers.Finger_2 = 60;
    fingers.fingers.Finger_3 = 60;
    action_client.sendGoal(fingers);
    wait_for_fingers_stopped();
}


void JacoCustom::move_up(double distance){
    actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> action_client("/jaco/arm_pose",true);
    action_client.waitForServer();
    jaco_msgs::ArmPoseGoal pose_goal = jaco_msgs::ArmPoseGoal();

    arm_mutex.lock();
    pose_goal.pose = this->arm_pose;
    pose_goal.pose.header.frame_id = "/jaco_api_origin";
    pose_goal.pose.pose.position.z += distance;
    arm_mutex.unlock();

    action_client.sendGoal(pose_goal);
    wait_for_arm_stopped();
}

void JacoCustom::moveToPoint(double x, double y, double z, double rotx, double roty, double rotz, double rotw){
    actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> action_client("/jaco/arm_pose",true);
    action_client.waitForServer();
    jaco_msgs::ArmPoseGoal pose_goal = jaco_msgs::ArmPoseGoal();

    pose_goal.pose.header.frame_id = "/jaco_api_origin";
    pose_goal.pose.pose.position.x = x;
    pose_goal.pose.pose.position.y = y;
    pose_goal.pose.pose.position.z = z;
    pose_goal.pose.pose.orientation.x = rotx;
    pose_goal.pose.pose.orientation.y = roty;
    pose_goal.pose.pose.orientation.z = rotz;
    pose_goal.pose.pose.orientation.w = rotw;
    action_client.sendGoal(pose_goal);

    wait_for_arm_stopped();
}


geometry_msgs::PoseStamped JacoCustom::getArmPosition(){
    arm_mutex.lock();
    geometry_msgs::PoseStamped arm = arm_pose;
    arm_mutex.unlock();
    return arm;
}

jaco_msgs::FingerPosition JacoCustom::getFingersPosition(){
    fingers_mutex.lock();
    jaco_msgs::FingerPosition pose = fingers_pose;
    fingers_mutex.unlock();
    return pose;
}

geometry_msgs::PoseStamped JacoCustom::getGraspArmPosition(){
    // It is supposed that the fingers are open at the beginning and that the user is teaching the grasp
    // position when the fingers are closing

    jaco_msgs::FingerPosition old_pose = getFingersPosition();
    jaco_msgs::FingerPosition new_pose;

    // If the fingers are closed by more than threshold angle, open the fingers
    double treshold = 10.0;
    if(old_pose.Finger_1 > treshold || old_pose.Finger_2 > treshold || old_pose.Finger_3 > treshold){
        open_fingers();
    }

    bool cond = true;
    ros::Rate r(4);
    int closing_count = 0;
    double closed_threshold = 45;
    while(cond){
        r.sleep();
        new_pose = getFingersPosition();

        // break the loop if the count is bigger or equal than 5 or if the angle of the fingers are bigger than a certain angle (threshold)
        if(closing_count >= 5 || new_pose.Finger_1 > closed_threshold || new_pose.Finger_2 > closed_threshold || new_pose.Finger_3 > closed_threshold){
            cond = false;
        }
        // increment the counter if the angles of the fingers are bigger than the previous iteration
        else if(new_pose.Finger_1 > old_pose.Finger_1 || new_pose.Finger_2 > old_pose.Finger_2 || new_pose.Finger_3 > old_pose.Finger_3){
            closing_count++;
        }
        else{
            closing_count = 0;
        }
    }

    return getArmPosition();
}


bool JacoCustom::is_same_pose(geometry_msgs::PoseStamped* pose1, geometry_msgs::PoseStampedConstPtr pose2){
    bool cond1 = pose1->pose.position.x == pose2->pose.position.x;
    bool cond2 = pose1->pose.position.y == pose2->pose.position.y;
    bool cond3 = pose1->pose.position.z == pose2->pose.position.z;
    bool cond4 = pose1->pose.orientation.x == pose2->pose.orientation.x;
    bool cond5 = pose1->pose.orientation.y == pose2->pose.orientation.y;
    bool cond6 = pose1->pose.orientation.z == pose2->pose.orientation.z;
    bool cond7 = pose1->pose.orientation.w == pose2->pose.orientation.w;

    if(cond1 && cond2 && cond3 && cond4 && cond5 && cond6 && cond7){
        return true;
    }

    else{
        return false;
    }
}

bool JacoCustom::is_same_pose(jaco_msgs::FingerPosition* pose1, jaco_msgs::FingerPositionConstPtr pose2){
    bool cond1 = pose1->Finger_1 == pose2->Finger_1;
    bool cond2 = pose1->Finger_2 == pose2->Finger_2;
    bool cond3 = pose1->Finger_3 == pose2->Finger_3;

    if(cond1 && cond2 && cond3){
        return true;
    }

    else{
        return false;
    }
}


void JacoCustom::wait_for_arm_stopped(){
    arm_is_stopped_counter = 0;
    arm_is_stopped = false;
    check_arm_status = true;
    ros::Rate r(30);
    while(true){
        if(arm_is_stopped) break;
        else {r.sleep();}
    }
    std::cout << "Finished moving the arm!" << std::endl;
    check_arm_status = false;
}

void JacoCustom::wait_for_fingers_stopped(){
    fingers_are_stopped_counter = 0;
    fingers_are_stopped = false;
    check_fingers_status = true;
    ros::Rate r(30);
    while(true){
        if(fingers_are_stopped) break;
        else {r.sleep();}
    }
    std::cout << "Finished moving the fingers!" << std::endl;
    check_fingers_status = false;
}






