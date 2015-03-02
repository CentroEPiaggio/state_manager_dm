#ifndef ik_planning_substate_H
#define ik_planning_substate_H

#include <abstract_state.h>
#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include <dual_manipulation_shared/databasemapper.h>
#include <std_msgs/String.h>

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
    int plan_executed=0;
    bool initialized;
    ros::Subscriber lsub;
    ros::Subscriber rsub;
    ros::Subscriber bimanualsub;
    void callback_l(const std_msgs::String::ConstPtr& str);
    void callback_r(const std_msgs::String::ConstPtr& str);
    void callback_bimanual(const std_msgs::String::ConstPtr& str);
    void reset();
    
    databaseMapper db_mapper;
    bool plan_sent;
    bool failed;
};

#endif // ik_planning_substate_H
