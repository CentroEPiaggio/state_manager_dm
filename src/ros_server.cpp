#include "ros_server.h"
#include <ik_control_state.h>
#include <steady_state.h>
#include "transitions.h"
#include <getting_info_state.h>
#include <starting_state.h>
#include <semantic_planning_state.h>
#include <exit_state.h>

using namespace dual_manipulation::state_manager;

ros_server::ros_server()
{
    init();
    service = node.advertiseService("state_manager_ros_service", &ros_server::state_manager_ros_service, this);
    loop_thread=std::thread(&ros_server::loop,this);
}

void ros_server::init()
{
    auto starting=new starting_state(data);
    auto steady=new steady_state(data);
    auto getting_info=new getting_info_state(data);
    auto ready=new steady_state(data,"ready");
    auto planning=new semantic_planning_state(data);
    auto planned = new steady_state(data,"planned");
    auto moving=new ik_control_state(data);
    auto exiting=new exit_state(data);
    
    std::vector<std::tuple<abstract_state<transition>*,transition_type,abstract_state<transition>*>> transition_table{
        //------initial state---------+--------- command -----------------------------------+-- final state---- +
        std::make_tuple( starting     , std::make_pair(transition::started,true)            ,    steady         ),
        std::make_tuple( steady       , std::make_pair(transition::get_info,true)           ,    getting_info   ),
        std::make_tuple( getting_info , std::make_pair(transition::got_info,true)           ,    ready          ),
        std::make_tuple( ready        , std::make_pair(transition::plan,true)               ,    planning       ),
        std::make_tuple( ready        , std::make_pair(transition::get_info,true)           ,    getting_info   ),
        std::make_tuple( planning     , std::make_pair(transition::failed_plan,true)        ,    steady         ),
        std::make_tuple( planning     , std::make_pair(transition::good_plan,true)          ,    planned        ),
        std::make_tuple( planned      , std::make_pair(transition::abort_plan,true)         ,    steady         ),
        std::make_tuple( planned      , std::make_pair(transition::start_moving,true)       ,    moving         ),
        std::make_tuple( moving       , std::make_pair(transition::task_accomplished,true)  ,    steady         ),
        std::make_tuple( moving       , std::make_pair(transition::abort_move,true)         ,    steady         ),
        //----------------------------+-----------------------------------------------------+-------------------+
        //         std::make_tuple( planning     , std::make_pair(transition::planning_done,true)      ,    steady         ),
        //         std::make_tuple( steady       , std::make_pair(transition::start_moving,true)       ,    moving         ),
        std::make_tuple( starting     , std::make_pair(transition::exit,true)               ,    exiting           ),
        std::make_tuple( steady       , std::make_pair(transition::exit,true)               ,    exiting           ),
        std::make_tuple( getting_info , std::make_pair(transition::exit,true)               ,    exiting           ),
        std::make_tuple( ready        , std::make_pair(transition::exit,true)               ,    exiting           ),
        std::make_tuple( planning     , std::make_pair(transition::exit,true)               ,    exiting           ),
        std::make_tuple( planned      , std::make_pair(transition::exit,true)               ,    exiting           ),
        std::make_tuple( moving       , std::make_pair(transition::exit,true)               ,    exiting           ),
        //----------------------------+-----------------------------------------------------+-------------------+
        std::make_tuple( getting_info , std::make_pair(transition::failed,true)             ,    steady            )
    };
    sm.insert(transition_table);
    this->transition_table=transition_table;
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
//     std::cout<<current_state->get_type()<<std::endl;
    current_state->run();
    ros::spinOnce(); //Will check for user commands
    usleep(5000);
    if (current_state->isComplete())
    {
	auto temp_map = current_state->getResults();
	for (auto temp:temp_map)
	    transition_map[temp.first]=temp.second;
    }
    for (auto trigger: transition_map)
    {
	auto temp_state = sm.evolve_state_machine(current_state, trigger);
	if (temp_state!=current_state)
	{
	    current_state=temp_state;
            current_state->reset();
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

void ros_server::reset()
{
    std::map<abstract_state<transition>*,bool> deleted;
    data.reset();
    for (auto line:transition_table)
    {
	if(deleted.count(std::get<0>(line)))
	  continue;
	else
	{
	  delete (std::get<0>(line));
	  deleted[std::get<0>(line)] = true;
	}
    }
    init();
}

bool ros_server::state_manager_ros_service(dual_manipulation_shared::state_manager_service::Request &req, dual_manipulation_shared::state_manager_service::Response &res)
{
    res.ack=true;

    //NOTE: maybe some of this should be removed, for now they help to force transitions

    if(req.command == "reset") reset();
    
    if(req.command == "started") transition_map[transition::started]=true;
    else if(req.command == "get_info") transition_map[transition::get_info]=true;
    else if(req.command == "plan") transition_map[transition::plan]=true;
    else if(req.command == "got_info") transition_map[transition::got_info]=true;
    else if(req.command == "good_plan") transition_map[transition::good_plan]=true;
    else if(req.command == "abort_plan") transition_map[transition::abort_plan]=true;
    else if(req.command == "start_moving") transition_map[transition::start_moving]=true;
    else if(req.command == "abort_move") transition_map[transition::abort_move]=true;
    else if(req.command == "task_accomplished") transition_map[transition::task_accomplished]=true;
    else if(req.command == "exit") transition_map[transition::exit]=true;
    else res.ack = false;

    return true;
}

ros_server::~ros_server()
{
    current_state=new exit_state(data);
    if(loop_thread.joinable()) join();
}
