#ifndef IK_CONTROL_STATE_H
#define IK_CONTROL_STATE_H

#include "abstract_state.h"
#include "state_machine.hpp"
#include "dual_manipulation_shared/ik_service.h"
#include "ros/ros.h"

class ik_steady_substate : public abstract_state<ik_transition>
{
public:
    ik_steady_substate(ik_shared_memory& data){};
    inline virtual std::map< ik_transition, bool > getResults(){std::map< ik_transition, bool > results; results[ik_transition::plan]=true; return results;};
    virtual void run(){};
    virtual bool isComplete(){return true;};
    virtual std::string get_type(){return "ik_steady_substate";};
};

class ik_exiting_substate : public abstract_state<ik_transition>
{
public:
    ik_exiting_substate(ik_shared_memory& data){};
    inline virtual std::map< ik_transition, bool > getResults(){std::map< ik_transition, bool > results; return results;};
    virtual void run(){};
    virtual bool isComplete(){return true;};
    virtual std::string get_type(){return "ik_exiting_substate";};
};

class ik_failing_substate : public abstract_state<ik_transition>
{
public:
    ik_failing_substate(ik_shared_memory& data){};
    inline virtual std::map< ik_transition, bool > getResults(){std::map< ik_transition, bool > results; return results;};
    virtual void run(){};
    virtual bool isComplete(){return true;};
    virtual std::string get_type(){return "ik_failing_substate";};
};

class ik_control_state : public abstract_state<transition>
{
public:
    ik_control_state(shared_memory& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
    virtual void reset();
    private:
    bool result;
    bool complete;
    
private:
    dual_manipulation::state_manager::state_machine<abstract_state<ik_transition>*,ik_transition_type> sm;
    abstract_state<ik_transition>* current_state;
    std::map<ik_transition,bool> transition_map;
    ik_steady_substate* waiting;
    shared_memory& data_;
    ik_shared_memory subdata;
    void fake_plan();
    void print_plan();
    void show_plan_with_tf();
    void show_plan_with_markers();
    ros::NodeHandle n;
    ros::ServiceClient client;
    dual_manipulation_shared::ik_service srv;
    // these bool will be updated from other states (such as ik_checking_grasp...) and from the GUI, respectively
    bool new_plan, show_plan;
    // whether we need to ask for another semantic planning...
    bool need_replan;
    const databaseMapper& db_mapper;
    ros::Publisher planned_path_publisher_;
    ros::ServiceClient scene_object_client;
    ros::ServiceClient scene_client_;
};

#endif // IK_CONTROL_STATE_H
