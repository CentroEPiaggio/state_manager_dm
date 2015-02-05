#include "moving_state.h"
#include "ros_server.h"

moving_state::moving_state(shared_memory& data)
{
    
}


std::map< transition, bool > moving_state::getResults()
{

}

bool moving_state::isComplete()
{
    return false;
}

void moving_state::run()
{

}

std::string moving_state::get_type()
{
    return "moving_state";
}