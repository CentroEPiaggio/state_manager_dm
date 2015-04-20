#include "../include/steady_state.h"

steady_state::steady_state(shared_memory& data,std::string type):data_(data)
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

void steady_state::reset()
{
    data_.filtered_source_nodes.clear();
    data_.filtered_target_nodes.clear();
    data_.cartesian_plan.clear();
}
