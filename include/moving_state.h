#ifndef MOVING_STATE_H
#define MOVING_STATE_H

#include "abstract_state.h"
#include "transitions.h"

class moving_state : public abstract_state<transition>
{
public:
moving_state();
bool isComplete();
void run();
std::map<transition,bool> getResults();

};

#endif // MOVING_STATE_H
