#ifndef ik_moving_substate_H
#define ik_moving_substate_H

#include "abstract_state.h"
#include "transitions.h"
#include "ros_server.h"
#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"

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
    int seq;
};

#endif // ik_moving_substate_H
