#ifndef STEADY_STATE_H
#define STEADY_STATE_H

#include "abstract_state.h"

class steady_state : public abstract_state<transition>
{
public:
    steady_state(shared_memory& data,std::string type="steady_state");
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
    private:
    std::string type;
};

#endif // STEADY_STATE_H
