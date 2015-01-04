#ifndef PLANNING_STATE_H
#define PLANNING_STATE_H

#include <abstract_state.h>

class planning_state : public abstract_state<transition>
{
public:
    planning_state(shared_memory& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
};

#endif // PLANNING_STATE_H
