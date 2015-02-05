#ifndef STARTING_STATE_H
#define STARTING_STATE_H

#include "abstract_state.h"

class starting_state : public abstract_state<transition>
{
public:
    starting_state(shared_memory& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
};

#endif // STARTING_STATE_H
