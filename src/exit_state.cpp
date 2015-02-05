#include "../include/exit_state.h"

exit_state::exit_state(shared_memory& data)
{

}

std::map< transition, bool > exit_state::getResults()
{

}

void exit_state::run()
{

}

bool exit_state::isComplete()
{
    return false;
}

std::string exit_state::get_type()
{
    return "exit_state";
}