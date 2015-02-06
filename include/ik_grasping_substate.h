#ifndef IK_GRASPING_SUBSTATE_H
#define IK_GRASPING_SUBSTATE_H

#include "abstract_state.h"
#include "transitions.h"

class ik_grasping_substate : public abstract_state<ik_transition>
{
public:
    ik_grasping_substate(ik_shared_memory& data);
    bool isComplete();
    void run();
    std::map<ik_transition,bool> getResults();
    virtual std::string get_type();
private:

};

#endif // IK_GRASPING_SUBSTATE_H
