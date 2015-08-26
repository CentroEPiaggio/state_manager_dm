#ifndef IK_NEED_SEMANTIC_REPLAN_H
#define IK_NEED_SEMANTIC_REPLAN_H

#include "abstract_state.h"

class ik_need_semantic_replan : public abstract_state<ik_transition>
{
public:
    ik_need_semantic_replan(ik_shared_memory& subdata, shared_memory& data);
    virtual std::map< ik_transition, bool > getResults();
    virtual void run(){ask_semantic_replan();};
    virtual bool isComplete(){return true;};
    virtual std::string get_type(){return "ik_need_semantic_replan";};
    virtual void reset();
    void ask_semantic_replan();
private:
    ik_shared_memory& subdata_;
    shared_memory& data_;
    bool need_replan_;
    bool failed_;
    const databaseMapper& database_;
    int get_grasp_id_from_db(int object_id, const geometry_msgs::Pose& pose, int ee_id);
};

#endif // IK_NEED_SEMANTIC_REPLAN_H
