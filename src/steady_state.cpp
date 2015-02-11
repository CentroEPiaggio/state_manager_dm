#include "../include/steady_state.h"

steady_state::steady_state(shared_memory& data,std::string type)
{
this->type=type;
}

std::map< transition, bool > steady_state::getResults()
{

}

void steady_state::run()
{
    usleep(200000);
}

bool steady_state::isComplete()
{
    return false;
}

std::string steady_state::get_type()
{
    return type;
}