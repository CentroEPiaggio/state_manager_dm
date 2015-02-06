#include "../include/ik_grasping_substate.h"

ik_grasping_substate::ik_grasping_substate(ik_shared_memory& data)
{

}

std::map< ik_transition, bool > ik_grasping_substate::getResults()
{
    std::map< ik_transition, bool > results;
    return results;
}

void ik_grasping_substate::run()
{

}

bool ik_grasping_substate::isComplete()
{
    return false;
}

std::string ik_grasping_substate::get_type()
{
    return "ik_grasping_substate";
}