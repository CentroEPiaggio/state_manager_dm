#ifndef MOVING_STATE_H
#define MOVING_STATE_H

#include "abstract_state.h"
#include "transitions.h"
#include "ros_server.h"

class moving_state : public abstract_state<transition>
{
public:
    moving_state(shared_memory& data);
    bool isComplete();
    void run();
    std::map<transition,bool> getResults();
};

#endif // MOVING_STATE_H
