#include "../include/ik_checking_grasp_substate.h"

ik_checking_grasp_substate::ik_checking_grasp_substate(ik_shared_memory& data)
{

}

std::map< ik_transition, bool > ik_checking_grasp_substate::getResults()
{
    std::map< ik_transition, bool > results;
    return results;
}

void ik_checking_grasp_substate::run()
{

}

bool ik_checking_grasp_substate::isComplete()
{
    return false;
}

std::string ik_checking_grasp_substate::get_type()
{
    return "ik_checking_grasp_substate";
}