#ifndef PLANNING_STATE_H
#define PLANNING_STATE_H

#include <abstract_state.h>
#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"

class planning_state : public abstract_state<transition>
{
public:
    planning_state(shared_memory& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
private:
    shared_memory& data_;
    ros::NodeHandle n;
    ros::ServiceClient client;
    dual_manipulation_shared::ik_service srv;
    bool plan_executed;
};

#endif // PLANNING_STATE_H
