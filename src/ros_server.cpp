#include "ros_server.h"
#include <moving_state.h>
#include <steady_state.h>
#include "transitions.h"
#include <getting_info_state.h>
#include <starting_state.h>
#include <planning_state.h>

using namespace dual_manipulation::state_manager;

ros_server::ros_server()
{
    auto moving=new moving_state(data);
    auto steady=new steady_state(data);
    auto planning=new planning_state(data);
    auto starting=new starting_state(data);
    auto getting_info=new getting_info_state(data);
    
    std::vector<std::tuple<abstract_state<transition>*,transition_type,abstract_state<transition>*>> transition_table{
        //------initial state---------+--------- command -----------------------------------+-- final state---- +
        std::make_tuple( starting     , std::make_pair(transition::started,true)            ,    steady         ),
        std::make_tuple( steady       , std::make_pair(transition::get_info,true)           ,    getting_info   ),
        std::make_tuple( steady       , std::make_pair(transition::plan,true)               ,    planning       ),
        std::make_tuple( getting_info , std::make_pair(transition::got_info,true)           ,    steady         ),
        std::make_tuple( planning     , std::make_pair(transition::abort_plan,true)         ,    steady         ),
        std::make_tuple( planning     , std::make_pair(transition::start_moving,true)       ,    moving         ),
        std::make_tuple( moving       , std::make_pair(transition::task_accomplished,true)  ,    steady         )
        //--------------------------+-----------------------------------------------------+-------------------+
    };
    sm.insert(transition_table);
    
    service = node.advertiseService("state_manager_ros_service", &ros_server::state_manager_ros_service, this);
    loop_thread=std::thread(&ros_server::loop,this);
    current_state=steady;
}

void ros_server::loop()
{
    /*Rules:
     *
     * Current state -> Run
     * Check user commands 
     * If the current state is complete
     * . Check current state results
     * . For each transition
     *   - try to change state
     *   - if the state changed, drop all the other transitions
     *   - else check next transition
     * Else do nothing
     */

    while(current_state->get_type() != "exit_state")
    {
    current_state->run();
    ros::spinOnce(); //Will check for user commands
    if (current_state->isComplete())
    {
	auto temp_map = current_state->getResults(); //TODO save them
	for (auto temp:temp_map)
	    transition_map[temp.first]=temp.second;
    }
    for (auto trigger: transition_map)
    {
	auto temp_state = sm.evolve_state_machine(current_state, trigger);
	if (temp_state!=current_state)
	{
	    current_state=temp_state;
	    std::cout<<"- new state type: "<<current_state->get_type()<<std::endl;
	    transition_map.clear();
	    break;
	}
    }
    }
}

void ros_server::join()
{
    return loop_thread.join();
}


bool ros_server::state_manager_ros_service(dual_manipulation_shared::state_manager_service::Request &req, dual_manipulation_shared::state_manager_service::Response &res)
{
    //TODO: save user commands as transitions into the transition_map
    res.ack=true;
    return true;
}

ros_server::~ros_server()
{
    join();
}