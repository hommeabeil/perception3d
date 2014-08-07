#include <ros/ros.h>
#include <jaco_msgs/JointVelocity.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_publisher_joint_test");

    ros::NodeHandle nh;
    ros::NodeHandle nParam("~");

    int jointName;
    nParam.param("joint_name", jointName, 1);

    int jointVelocity2;
    nParam.param("joint_velocity2", jointVelocity2, 0);

    int jointVelocity;
    nParam.param("joint_velocity", jointVelocity, 0);

    ros::Publisher pub = nh.advertise<jaco_msgs::JointVelocity>("/jaco_arm_driver/in/joint_velocity",1);
    ros::Rate r(1000);

    jaco_msgs::JointVelocity sendMessageZero;
    sendMessageZero.joint1 = 0;
    sendMessageZero.joint2 = 0;
    sendMessageZero.joint3 = 0;
    sendMessageZero.joint4 = 0;
    sendMessageZero.joint5 = 0;
    sendMessageZero.joint6 = 0;

    jaco_msgs::JointVelocity sendMessageParam1;

    if(jointName = 1)
    {
        sendMessageParam1.joint1 =jointVelocity;
        sendMessageParam1.joint2 =0;
        sendMessageParam1.joint3 =0;
        sendMessageParam1.joint4 =0;
        sendMessageParam1.joint5 =0;
        sendMessageParam1.joint6 =0;
    }
    else if(jointName = 2)
    {
        sendMessageParam1.joint1 =0;
        sendMessageParam1.joint2 =jointVelocity;
        sendMessageParam1.joint3 =0;
        sendMessageParam1.joint4 =0;
        sendMessageParam1.joint5 =0;
        sendMessageParam1.joint6 =0;
    }
    else if(jointName = 3)
    {
        sendMessageParam1.joint1 =0;
        sendMessageParam1.joint2 =0;
        sendMessageParam1.joint3 =jointVelocity;
        sendMessageParam1.joint4 =0;
        sendMessageParam1.joint5 =0;
        sendMessageParam1.joint6 =0;
    }
    else if(jointName = 4)
    {
        sendMessageParam1.joint1 =0;
        sendMessageParam1.joint2 =0;
        sendMessageParam1.joint3 =0;
        sendMessageParam1.joint4 =jointVelocity;
        sendMessageParam1.joint5 =0;
        sendMessageParam1.joint6 =0;
    }
   else if(jointName = 5)
    {
        sendMessageParam1.joint1 =0;
        sendMessageParam1.joint2 =0;
        sendMessageParam1.joint3 =0;
        sendMessageParam1.joint4 =0;
        sendMessageParam1.joint5 =jointVelocity;
        sendMessageParam1.joint6 =0;
    }
    else if(jointName = 6)
    {
        sendMessageParam1.joint1 =0;
        sendMessageParam1.joint2 =0;
        sendMessageParam1.joint3 =0;
        sendMessageParam1.joint4 =0;
        sendMessageParam1.joint5 =0;
        sendMessageParam1.joint6 =jointVelocity;
    }


    jaco_msgs::JointVelocity sendMessageParam2;
    if(jointName = 1)
    {
        sendMessageParam2.joint1 =jointVelocity2;
        sendMessageParam2.joint2 =0;
        sendMessageParam2.joint3 =0;
        sendMessageParam2.joint4 =0;
        sendMessageParam2.joint5 =0;
        sendMessageParam2.joint6 =0;
    }
    else if(jointName = 2)
    {
        sendMessageParam2.joint1 =0;
        sendMessageParam2.joint2 =jointVelocity2;
        sendMessageParam2.joint3 =0;
        sendMessageParam2.joint4 =0;
        sendMessageParam2.joint5 =0;
        sendMessageParam2.joint6 =0;
    }
    else if(jointName = 3)
    {
        sendMessageParam2.joint1 =0;
        sendMessageParam2.joint2 =0;
        sendMessageParam2.joint3 =jointVelocity2;
        sendMessageParam2.joint4 =0;
        sendMessageParam2.joint5 =0;
        sendMessageParam2.joint6 =0;
    }
    else if(jointName = 4)
    {
        sendMessageParam2.joint1 =0;
        sendMessageParam2.joint2 =0;
        sendMessageParam2.joint3 =0;
        sendMessageParam2.joint4 =jointVelocity2;
        sendMessageParam2.joint5 =0;
        sendMessageParam2.joint6 =0;
    }
    else if(jointName = 5)
    {
        sendMessageParam2.joint1 =0;
        sendMessageParam2.joint2 =0;
        sendMessageParam2.joint3 =0;
        sendMessageParam2.joint4 =0;
        sendMessageParam2.joint5 =jointVelocity2;
        sendMessageParam2.joint6 =0;
    }
    else if(jointName = 6)
    {
        sendMessageParam2.joint1 =0;
        sendMessageParam2.joint2 =0;
        sendMessageParam2.joint3 =0;
        sendMessageParam2.joint4 =0;
        sendMessageParam2.joint5 =0;
        sendMessageParam2.joint6 =jointVelocity2;
    }



    ros::Time duration2 = ros::Time::now() + ros::Duration(2,0);
    while(ros::Time::now() < duration2)
    {
        pub.publish(sendMessageZero);
        r.sleep();
    }

    ros::Time duration22 = ros::Time::now() + ros::Duration(2,0);
    while(ros::Time::now() < duration22)
    {
        pub.publish(sendMessageParam1);
        r.sleep();
    }

    ros::Time duration222 = ros::Time::now() + ros::Duration(2,0);
    while(ros::Time::now() < duration222)
    {
        pub.publish(sendMessageParam2);
        r.sleep();
    }

    ros::Time duration2222 = ros::Time::now() + ros::Duration(2,0);
    while(ros::Time::now() < duration2222)
    {
        pub.publish(sendMessageZero);
        r.sleep();
    }






    return 0;
}
