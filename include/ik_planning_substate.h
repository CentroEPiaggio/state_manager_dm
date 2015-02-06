#ifndef ik_planning_substate_H
#define ik_planning_substate_H

#include <abstract_state.h>
#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"

class ik_planning_substate : public abstract_state<ik_transition>
{
public:
    ik_planning_substate(ik_shared_memory& data);
    virtual std::map< ik_transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
private:
    ik_shared_memory& data_;
    ros::NodeHandle n;
    ros::ServiceClient client;
    dual_manipulation_shared::ik_service srv;
    bool plan_executed;
    int seq;
    bool initialize;
};

#endif // ik_planning_substate_H
