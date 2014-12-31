#include <ros/ros.h>
#include <state_machine.hpp>
#include <abstract_state.h>
#include <transitions.h>
#include <moving_state.h>
#include <steady_state.h>

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }
    dual_manipulation::state_manager::state_machine<abstract_state<transition>*,transition_type> sm;
    sm.insert(std::make_tuple(new moving_state(),std::make_pair(transition::task_accomplished,true),new steady_state()));
    return 0;
}
