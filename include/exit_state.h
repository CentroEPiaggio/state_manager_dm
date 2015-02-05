#ifndef EXIT_STATE_H
#define EXIT_STATE_H

#include "abstract_state.h"

class exit_state : public abstract_state<transition>
{
public:
    exit_state(shared_memory& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
};

#endif // EXIT_STATE_H
