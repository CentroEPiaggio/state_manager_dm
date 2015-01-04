#ifndef ABSTRACT_STATE_H
#define ABSTRACT_STATE_H
#include <map>
#include "transitions.h"
#include "shared_memory.h"

template <class property>
class abstract_state
{
private:

public:
    abstract_state(){
    }
    virtual bool isComplete()=0;
    virtual void run()=0;
    virtual std::map<property,bool> getResults()=0;
};

#endif //ABSTRACT_STATE_H