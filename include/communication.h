#ifndef communication_H
#define communication_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <objectExtractor.h>
#include <stdlib.h>
#include <fileAPI.h>
#include <jaco_custom.h>

class Communication
{
public:

    Communication(ObjectExtractor *p_obj_e, FileAPI *p_api, JacoCustom *p_jaco);
    void callback_android_listener(const std_msgs::String& p_input);
    void coordinate_processing(std_msgs::String p_coordinate);
    void grasp_processing(std_msgs::String p_grasp);
    void train_processing(std_msgs::String p_train);
    void spin_once();

    bool get_coordinate_received() const;
    bool get_grasp_received() const;
    bool get_train_received() const;

    void train();


private:

    ObjectExtractor* m_object_ex_ptr;
    FileAPI* m_api_ptr;
    JacoCustom* m_jaco_ptr;

    float m_coordinate_user_sended[2];
    int m_position_vector_cvfh;

    bool m_coordinate_received;
    bool m_grasp_received;
    bool m_train_received;

};

#endif
