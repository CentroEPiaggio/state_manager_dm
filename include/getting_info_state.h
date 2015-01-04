#ifndef GETTING_INFO_STATE_H
#define GETTING_INFO_STATE_H

#include <abstract_state.h>

class getting_info_state : public abstract_state<transition>
{
public:
    getting_info_state(shared_memory& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
};

#endif // GETTING_INFO_STATE_H
