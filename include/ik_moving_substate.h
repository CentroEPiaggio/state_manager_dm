#ifndef ik_moving_substate_H
#define ik_moving_substate_H

#include "abstract_state.h"
#include "transitions.h"
#include "ros_server.h"
#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/ik_response.h"
#include <dual_manipulation_shared/databasemapper.h>
#include <std_msgs/String.h>
#include <mutex>

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
    ros::Subscriber exe_sub,grasp_sub,ungrasp_sub;
    bool initialized;
    void callback(const dual_manipulation_shared::ik_response::ConstPtr& str, std::string type);
    bool parallelize_planning;
    
    void reset();
    
    const databaseMapper& db_mapper;
    bool move_sent;
    /// true if a command which needs to be executed alone is being executed
    bool alone_execution;
    int moving_executed;
    bool failed;
    int sequence_counter;
    std::set<int> pending_sequence_numbers;
    std::mutex moving_executed_mutex;
    moving_cmd commands;
};

#endif // ik_moving_substate_H
