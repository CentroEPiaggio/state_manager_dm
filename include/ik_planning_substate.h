#ifndef ik_planning_substate_H
#define ik_planning_substate_H

#include <abstract_state.h>
#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include <dual_manipulation_shared/ik_response.h>
#include <dual_manipulation_shared/databasemapper.h>
#include <std_msgs/String.h>
#include <mutex>

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
    ros::Subscriber plan_sub;
    void callback(const dual_manipulation_shared::ik_response::ConstPtr& str, std::string type);
    void reset();
    int sequence_counter;
    std::set<int> pending_sequence_numbers;
    std::mutex plan_executed_mutex;
    const databaseMapper& db_mapper;
    bool plan_sent;
    bool failed;
    bool checking_grasp;
    planning_cmd commands;
};

#endif // ik_planning_substate_H
