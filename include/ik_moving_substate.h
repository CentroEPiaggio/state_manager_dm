#ifndef ik_moving_substate_H
#define ik_moving_substate_H

#include "abstract_state.h"
#include "transitions.h"
#include "ros_server.h"
#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include <std_msgs/String.h>

class ik_moving_substate : public abstract_state<ik_transition>
{
public:
    ik_moving_substate(ik_shared_memory& data);
    bool isComplete();
    void run();
    std::map<ik_transition,bool> getResults();
    virtual std::string get_type();
private:
    ik_shared_memory& data_;
    ros::NodeHandle n;
    ros::ServiceClient client;
    dual_manipulation_shared::ik_service srv;
    bool motion_executed;
    ros::Subscriber lsub;
    ros::Subscriber rsub;
    ros::Subscriber bimanualsub;
    bool initialized;
    void callback_l(const std_msgs::String::ConstPtr& str);
    void callback_r(const std_msgs::String::ConstPtr& str);
    void callback_bimanual(const std_msgs::String::ConstPtr& str);
};

#endif // ik_moving_substate_H
