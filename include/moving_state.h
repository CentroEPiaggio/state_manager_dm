#ifndef MOVING_STATE_H
#define MOVING_STATE_H

#include "abstract_state.h"
#include "transitions.h"
#include "ros_server.h"
#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"

class moving_state : public abstract_state<transition>
{
public:
    moving_state(shared_memory& data);
    bool isComplete();
    void run();
    std::map<transition,bool> getResults();
    virtual std::string get_type();
private:
    shared_memory& data_;
    ros::NodeHandle n;
    ros::ServiceClient client;
    dual_manipulation_shared::ik_service srv;
    bool motion_executed;
    int seq;
};

#endif // MOVING_STATE_H
